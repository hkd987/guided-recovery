# Real flight logs for gate 3

Drop host flight-computer CSV exports here (one flight per file). `grreplay`
picks up every `*.csv` in this folder on every CI run, together with the
public set in [../public](../public/README.md).

Accepted columns (header row, case-insensitive; a header written as a `#`
comment, as AltosUI does, is fine; units in brackets such as `height (ft)` or
`pressure (hPa)` are converted):

| column | meaning |
|---|---|
| `time` | seconds |
| `acceleration` or `accel` | m/s², along the airframe axis, gravity removed (AltOS convention: 0 on the pad) |
| `accel_x`, `accel_y`, `accel_z` (optional) | 3-axis IMU, m/s² with gravity; lets the replay give the pod the true magnitude when the airframe hangs sideways |
| `pressure` (Pa or hPa) **or** `height` / `altitude` (m) | barometer |
| `state_name` or `state` | host flight state; the first `main` row marks the main deployment, a `drogue` state marks dual deploy |
| `main_t` (optional) | deployment time in seconds, if the log has no state column |

AltOS exports from AltosUI (TeleMetrum, TeleMega, EasyMega, EasyMini) work
as-is. EasyMini has no accelerometer: AltosUI fills its acceleration column
from the barometer, so those logs show no opening shocks and the replay
allows them a longer window.

Other formats read as-is:

| Flight computer | What to drop in | Deployment truth | Accelerometer |
|---|---|---|---|
| Featherweight **Blue Raven** | the low-rate file (`Flight_Time_(s)`, `Baro_Press_(atm)`, event flags) **and** its high-rate file in the same folder, named the same with `LR` -> `HR` (or `low_rate`/`lowrate`/`low-rate` -> `high...`) | first `Main_fired` row; a unit whose main never fired is skipped | high-rate `Accel_X/Y/Z` in g, 500 Hz; the axis carrying gravity on the pad is the airframe axis |
| Missile Works **RRC3** | the CSV export (`Time,Altitude,Pressure,...,Events,...`) | the `Main` event | none: the replay feeds the pod barometric acceleration, so no opening jolts |
| **Eggtimer** (Quantum, Quasar, TRS) | the CSV (`T,Alt,Veloc,FAlt,FVeloc,...,Drogue,Main`; Alt in feet) | `Main` column turning nonzero | none, as RRC3 |

Pass the Blue Raven low-rate file; a high-rate file given on its own is
skipped. If its high-rate sibling is missing, the replay runs on the
barometer alone. Some exports hold the flight twice; only the first copy is
read. Other brands: rename columns to match the table above.

Only add logs you have permission to share. The three logs here now are
MIT-licensed; see [THIRD_PARTY.md](THIRD_PARTY.md).
