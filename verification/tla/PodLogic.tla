------------------------------ MODULE PodLogic ------------------------------
(***************************************************************************)
(* Formal model of the Guided Recovery pod's decision logic.               *)
(*                                                                         *)
(* One step of this model = one 5 ms call of FlightCore::update() on the   *)
(* pod, including the console requests the ESP32 target applies just       *)
(* before it (firmware/targets/esp32s3/src/main.cpp).                      *)
(*                                                                         *)
(* Modelled, operator by operator, from the C++:                           *)
(*   Detect         PhaseDetector::update      (phase_detector.cpp)        *)
(*   Update         FlightCore::update         (flight_core.cpp):          *)
(*                    updatePadReferences, stage->phase mapping,           *)
(*                    enterCanopy, checkSafety, enterSpiral, motor power,  *)
(*                    winch fault latching                                 *)
(*                                                                         *)
(* Everything the sensors and timers decide (is the acceleration high      *)
(* enough, has the descent been steady for 3 s, has GPS been silent for    *)
(* 2 s, ...) is an input that TLC chooses freely at every step. So each    *)
(* property below holds for EVERY possible sequence of sensor readings,    *)
(* faults and console commands, not just the ones a simulator produces.    *)
(* What this model does not cover: the numeric thresholds themselves and   *)
(* the guidance / estimation maths (those are what gates 1-3 test).        *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets, TLC

CONSTANT Mutation
  \* "none" = the real logic. The other values each re-introduce one bug so
  \* we can see TLC catch it: "stale_candidate" (the bug this model found),
  \* "motors_in_descent", "home_unfrozen", "no_gps_failsafe", "disarm_in_flight",
  \* "resume_steering" (a restart under canopy goes straight back to guiding).

Stages == {"Disarmed", "Pad", "Ascent", "Descent", "Canopy", "Landed"}
Phases == {"Disarmed", "Pad", "Ascent", "Descent", "Guided", "Spiral", "Neutral", "Landed"}
FaultNames == {"GpsLost", "Fence", "WinchL", "WinchR", "Lopsided", "Collapse",
               "LowBattery", "NoHome", "LowDeploy", "Reset"}
SpiralReasons == {"GpsLost", "Fence", "WinchL", "WinchR", "Lopsided", "LowBattery", "NoHome"}
Homes == 0..2   \* 0 = no home point; 1 and 2 = two different averaged GPS positions
InFlight == {"Ascent", "Descent", "Canopy"}

VARIABLE s     \* the pod's abstract state, one record

InitState == [stage  |-> "Disarmed",  \* PhaseDetector stage
              phase  |-> "Disarmed",  \* FlightCore phase (what the pod is doing)
              cand   |-> FALSE,       \* PhaseDetector::candidate_ (launch suspected)
              home   |-> 0,           \* saved home point
              bench  |-> FALSE,       \* FlightCore::benchActive_
              motors |-> FALSE,       \* Outputs::motorsEnabled
              steer  |-> FALSE,       \* Status::steeringActive (brakes commanded)
              faults |-> {},          \* Status::faults (latched)
              hazard |-> FALSE,       \* ghost: a guidance-ending hazard was present this step
              rc     |-> FALSE]       \* ghost: the processor restarted under canopy this flight

-----------------------------------------------------------------------------
(* Inputs. Only the inputs the code actually reads in a given stage vary; *)
(* the rest are fixed, which keeps TLC fast without hiding behaviours.     *)

B == BOOLEAN
NoInput == [armed |-> FALSE, benchReq |-> FALSE, reset |-> FALSE,
            accel |-> FALSE, baroConfirm |-> FALSE, candTimeout |-> FALSE,
            baroLaunch |-> FALSE, homeFix |-> 0, apogee |-> FALSE,
            steady |-> FALSE, landed |-> FALSE, lowDeploy |-> FALSE,
            gps |-> FALSE, fence |-> FALSE, lopsided |-> FALSE, lowBatt |-> FALSE,
            collapse |-> FALSE, stallL |-> FALSE, stallR |-> FALSE,
            landedTimeout |-> FALSE]

InputsFor(x) ==
  CASE x.stage = "Disarmed" ->
         { [NoInput EXCEPT !.armed = a, !.benchReq = br, !.reset = rs, !.stallL = sl, !.stallR = sr]
             : a \in B, br \in B, rs \in B, sl \in B, sr \in B }
    [] x.stage = "Pad" ->
         { [NoInput EXCEPT !.armed = a, !.accel = ac, !.baroConfirm = bc, !.candTimeout = ct,
                           !.baroLaunch = bl, !.homeFix = h]
             : a \in B, ac \in B, bc \in B, ct \in B, bl \in B, h \in Homes }
    [] x.stage = "Ascent" ->
         { [NoInput EXCEPT !.armed = a, !.apogee = ap, !.homeFix = h]
             : a \in B, ap \in B, h \in (IF Mutation = "home_unfrozen" THEN Homes ELSE {0}) }
    [] x.stage = "Descent" ->
         { [NoInput EXCEPT !.armed = a, !.steady = st, !.landed = ld, !.lowDeploy = lo,
                           !.gps = g, !.fence = f, !.lopsided = lp, !.lowBatt = lb,
                           !.collapse = c, !.stallL = sl, !.stallR = sr]
             : a \in B, st \in B, ld \in B, lo \in B, g \in B, f \in B, lp \in B,
               lb \in B, c \in B, sl \in B, sr \in B }
    [] x.stage = "Canopy" ->
         { [NoInput EXCEPT !.armed = a, !.landed = ld, !.gps = g, !.fence = f,
                           !.lopsided = lp, !.lowBatt = lb, !.collapse = c,
                           !.stallL = sl, !.stallR = sr]
             : a \in B, ld \in B, g \in B, f \in B, lp \in B, lb \in B, c \in B,
               sl \in B, sr \in B }
    [] x.stage = "Landed" ->
         { [NoInput EXCEPT !.armed = a, !.landedTimeout = lt, !.stallL = sl, !.stallR = sr]
             : a \in B, lt \in B, sl \in B, sr \in B }

-----------------------------------------------------------------------------
(* PhaseDetector::update. Returns the new stage and candidate flag. *)

Detect(r, in) ==
  CASE r.stage = "Disarmed" ->
         [stage |-> IF in.armed THEN "Pad" ELSE "Disarmed", cand |-> r.cand]
    [] r.stage = "Pad" ->
         IF ~in.armed THEN [stage |-> "Disarmed", cand |-> FALSE]
         ELSE LET c1   == r.cand \/ in.accel             \* sustained >3 g raises the candidate
                  asc1 == c1 /\ in.baroConfirm           \* ...confirmed by +10 m on the baro
                  c2   == IF c1 /\ ~asc1 /\ r.cand /\ in.candTimeout THEN FALSE ELSE c1
                  asc  == asc1 \/ in.baroLaunch          \* baro-only launch path
              IN [stage |-> IF asc THEN "Ascent" ELSE "Pad", cand |-> c2]
    [] r.stage \in InFlight /\ Mutation = "disarm_in_flight" /\ ~in.armed ->
         [stage |-> "Disarmed", cand |-> FALSE]
    [] r.stage = "Ascent" ->
         [stage |-> IF in.apogee THEN "Descent" ELSE "Ascent", cand |-> r.cand]
    [] r.stage = "Descent" ->
         \* canopy check first, then the landed check in the same call
         LET st1 == IF in.steady THEN "Canopy" ELSE "Descent"
         IN [stage |-> IF in.landed THEN "Landed" ELSE st1, cand |-> r.cand]
    [] r.stage = "Canopy" ->
         [stage |-> IF in.landed THEN "Landed" ELSE "Canopy", cand |-> r.cand]
    [] r.stage = "Landed" ->
         IF in.armed THEN [stage |-> "Landed", cand |-> r.cand]
         ELSE [stage |-> "Disarmed",
               cand  |-> IF Mutation = "stale_candidate" THEN r.cand ELSE FALSE]

Spiral(f, why) == [phase |-> "Spiral", faults |-> f \cup {why}]

-----------------------------------------------------------------------------
(* FlightCore::update, plus the console requests applied just before it. *)

Update(x, in) ==
  LET \* Console: 'zero' / settings change rebuild the core; bench jog. Disarmed only.
      r     == IF in.reset /\ x.phase = "Disarmed" THEN InitState ELSE x
      b     == IF in.benchReq /\ r.phase = "Disarmed" THEN TRUE ELSE r.bench
      \* updatePadReferences: runs before the detector, sees the previous stage.
      home1 == CASE r.stage = "Disarmed" -> 0
                 [] (r.stage = "Pad" \/ (Mutation = "home_unfrozen" /\ r.stage = "Ascent"))
                      /\ (~r.cand \/ Mutation = "home_unfrozen") /\ in.homeFix # 0 -> in.homeFix
                 [] OTHER -> r.home
      d     == Detect(r, in)
      chg   == d.stage # r.stage
      \* stage -> phase mapping, with enterCanopy
      ph1   == IF ~chg THEN r.phase
               ELSE CASE d.stage = "Canopy" ->
                           IF in.lowDeploy THEN "Neutral"
                           ELSE IF home1 = 0 THEN "Spiral" ELSE "Guided"
                      [] OTHER -> d.stage
      f1    == IF chg /\ d.stage = "Canopy"
               THEN r.faults \cup (IF in.lowDeploy THEN {"LowDeploy"}
                                   ELSE IF home1 = 0 THEN {"NoHome"} ELSE {})
               ELSE r.faults
      bench1 == IF chg /\ d.stage = "Pad" THEN FALSE ELSE b
      \* checkSafety (stall flags are latched from earlier steps)
      stL   == "WinchL" \in f1
      stR   == "WinchR" \in f1
      sf    == IF d.stage # "Canopy" \/ ph1 \in {"Neutral", "Landed"}
                 THEN [phase |-> ph1, faults |-> f1]
               ELSE IF in.collapse THEN [phase |-> "Neutral", faults |-> f1 \cup {"Collapse"}]
               ELSE IF ph1 # "Guided" THEN [phase |-> ph1, faults |-> f1]
               ELSE IF in.gps /\ Mutation # "no_gps_failsafe" THEN Spiral(f1, "GpsLost")
               ELSE IF in.fence    THEN Spiral(f1, "Fence")
               ELSE IF stL         THEN Spiral(f1, "WinchL")
               ELSE IF stR         THEN Spiral(f1, "WinchR")
               ELSE IF in.lopsided THEN Spiral(f1, "Lopsided")
               ELSE IF in.lowBatt  THEN Spiral(f1, "LowBattery")
               ELSE [phase |-> "Guided", faults |-> f1]
      ph    == sf.phase
      \* motor power
      mot   == CASE ph \in {"Guided", "Spiral", "Neutral"} -> TRUE
                 [] ph = "Landed" -> IF r.phase = "Landed" THEN r.motors /\ ~in.landedTimeout ELSE TRUE
                 [] ph = "Disarmed" -> bench1
                 [] ph = "Descent" /\ Mutation = "motors_in_descent" -> TRUE
                 [] OTHER -> FALSE
      \* winch stall detection only runs while the motors are powered
      f2    == sf.faults \cup (IF mot /\ in.stallL THEN {"WinchL"} ELSE {})
                         \cup (IF mot /\ in.stallR THEN {"WinchR"} ELSE {})
  IN [stage |-> d.stage, phase |-> ph, cand |-> d.cand, home |-> home1, bench |-> bench1,
      motors |-> mot, steer |-> ph \in {"Guided", "Spiral"}, faults |-> f2,
      hazard |-> (in.gps \/ in.fence \/ in.lopsided \/ in.lowBatt \/ in.collapse),
      rc |-> IF d.stage = "Disarmed" THEN FALSE ELSE r.rc]

(* A processor restart in flight (brownout, watchdog, panic), followed by    *)
(* FlightCore::resume() from the snapshot kept in RTC memory. No update has  *)
(* run yet, so the motors are still off.                                    *)
Resume(x) ==
  [stage  |-> x.stage,
   phase  |-> IF x.stage = "Canopy"
              THEN (IF Mutation = "resume_steering" /\ x.home # 0 THEN "Guided" ELSE "Neutral")
              ELSE x.stage,
   cand   |-> FALSE, home |-> x.home, bench |-> FALSE, motors |-> FALSE,
   steer  |-> FALSE,
   faults |-> x.faults \cup {"Reset"}, hazard |-> FALSE,
   rc     |-> x.rc \/ x.stage = "Canopy"]

-----------------------------------------------------------------------------
Init == s = InitState
WarmReset == s.stage \in InFlight /\ s' = Resume(s)
Next == (\E in \in InputsFor(s) : s' = Update(s, in)) \/ WarmReset

\* Fairness: time passes after landing (the 10 s line-release timer runs out).
LandedTick == s.phase = "Landed"
              /\ s' = Update(s, [NoInput EXCEPT !.armed = TRUE, !.landedTimeout = TRUE])
Spec == Init /\ [][Next]_s /\ WF_s(LandedTick)

-----------------------------------------------------------------------------
(* Safety invariants: true in every reachable state. *)

TypeOK ==
  /\ s.stage \in Stages /\ s.phase \in Phases /\ s.home \in Homes
  /\ s.cand \in B /\ s.bench \in B /\ s.motors \in B /\ s.steer \in B /\ s.hazard \in B
  /\ s.faults \subseteq FaultNames

\* The phase always agrees with the detector's stage.
PhaseMatchesStage ==
  /\ (s.phase = "Disarmed" <=> s.stage = "Disarmed")
  /\ (s.phase = "Pad" <=> s.stage = "Pad")
  /\ (s.phase = "Ascent" <=> s.stage = "Ascent")
  /\ (s.phase = "Descent" <=> s.stage = "Descent")
  /\ (s.phase \in {"Guided", "Spiral", "Neutral"} <=> s.stage = "Canopy")
  /\ (s.phase = "Landed" <=> s.stage = "Landed")

\* Guardrail 1: steering arms only after the canopy is confirmed open.
NoSteeringBeforeCanopy == s.steer => s.stage = "Canopy"

\* The winches are unpowered on the pad and inside the rocket...
MotorsOffInRocket == s.stage \in {"Pad", "Ascent", "Descent"} => ~s.motors
\* ...and only ever powered under a canopy, after landing, or on the bench.
MotorsOnlyWhenAllowed ==
  s.motors => (s.stage \in {"Canopy", "Landed"} \/ (s.phase = "Disarmed" /\ s.bench))

\* Guardrail 2: guiding needs a home point (saved on the pad; see HomeFrozenInFlight).
GuidedHasHome == s.phase = "Guided" => s.home # 0

\* Bench (console) motor control only exists while disarmed.
BenchOnlyWhenDisarmed == s.bench => s.phase = "Disarmed"

\* Guardrail 3: GPS loss, geofence, lopsided canopy, low battery or collapse
\* end guidance in the same 5 ms step they are detected.
NoGuidanceThroughHazard == s.phase = "Guided" => ~s.hazard

\* A spiral always has a recorded reason.
SpiralHasReason == s.phase = "Spiral" => s.faults \cap SpiralReasons # {}

\* Disarmed means no launch is pending.
DisarmedMeansNoLaunchPending == s.stage = "Disarmed" => ~s.cand

\* After a restart under canopy the pod never steers again that flight: it
\* cannot trust its rebuilt estimates, so it releases the brakes and rides.
NoSteeringAfterCanopyRestart == s.rc => ~s.steer

-----------------------------------------------------------------------------
(* Step properties: true of every transition. *)

\* The home point cannot change once a launch is suspected, through landing.
HomeFrozenInFlight ==
  [][ ((s.stage = "Pad" /\ s.cand) \/ s.stage \in InFlight \cup {"Landed"}) => s'.home = s.home ]_s

\* Every arming starts fresh: no pending launch, no home yet (it is learned again).
FreshArming == [][ (s.stage # "Pad" /\ s'.stage = "Pad") => (~s'.cand /\ s'.home = 0) ]_s

\* The magnet cannot disarm the pod in flight.
NoDisarmInFlight == [][ s.stage \in InFlight => s'.stage \in InFlight \cup {"Landed"} ]_s

\* Fallbacks latch: once spiralling or released, never back to guiding.
FallbacksLatch == [][ s.phase \in {"Spiral", "Neutral"} => s'.phase \in {"Spiral", "Neutral", "Landed"} ]_s

\* A jammed winch (latched on the bench or in flight) ends guidance at the next step.
StallEndsGuidance == [][ (s.phase = "Guided" /\ {"WinchL", "WinchR"} \cap s.faults # {}) => s'.phase # "Guided" ]_s

\* A collapse under canopy always releases the brakes.
CollapseReleases ==
  [][ (s.stage = "Canopy" /\ s'.stage = "Canopy" /\ "Collapse" \notin s.faults /\ "Collapse" \in s'.faults)
      => s'.phase = "Neutral" ]_s

(* Liveness: after landing the motors are eventually switched off. *)
LandedMotorsOff == (s.phase = "Landed") ~> (~s.motors \/ s.phase # "Landed")

-----------------------------------------------------------------------------
(* Witnesses: each should be VIOLATED, proving the state is reachable and the *)
(* properties above are not vacuously true.                                 *)
WitnessGuided         == s.phase # "Guided"
WitnessGpsSpiral      == ~(s.phase = "Spiral" /\ "GpsLost" \in s.faults)
WitnessCollapse       == ~(s.phase = "Neutral" /\ "Collapse" \in s.faults)
WitnessBenchJamFlies  == ~(s.phase = "Spiral" /\ s.faults = {"WinchL", "WinchR"})
WitnessNoHomeSpiral   == ~(s.phase = "Spiral" /\ s.faults = {"NoHome"})
WitnessLandedNoCanopy == ~(s.phase = "Landed" /\ s.faults = {} /\ s.motors)
WitnessArmedWithJam   == ~(s.phase = "Pad" /\ "WinchL" \in s.faults)
WitnessRestartRelease == ~(s.phase = "Neutral" /\ s.rc /\ s.motors)
WitnessRestartGuides  == ~(s.phase = "Guided" /\ s.faults = {"Reset"})
=============================================================================
