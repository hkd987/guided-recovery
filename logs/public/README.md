# Public real flight logs (fetched, not stored)

`manifest.tsv` lists 135 files of real flight data that university and hobby
rocketry teams have posted on GitHub:
- AltosUI exports from TeleMega, TeleMetrum, EasyMega and EasyMini;
- Featherweight Blue Raven low-rate and high-rate pairs (13 flights' worth);
- Missile Works RRC3 exports (barometer only).

They span 40 m to 8.3 km apogee, single and dual deploy, and include failed and
late mains. Gate 3 replays every one of them in CI: 122 logs once each Blue Raven
pair is read as one, plus the 3 in `logs/real/`.

Most of these repositories state no licence, so the files are **not copied
into this repository**. `tools/fetch_public_logs.sh` downloads them at test
time from the exact commit listed, and checks each file's sha256, so every run
replays the same bytes:

```sh
tools/fetch_public_logs.sh                 # -> build/public-logs/
./build/grreplay --main-alt log --min-flights 50 logs/real/*.csv build/public-logs/*.csv
```

Blue Raven high-rate files are picked up through their low-rate sibling (same
name, `LR` -> `HR`), so they must sit in the same folder; `grreplay` skips them
when they are passed on their own.

Thank you to the teams who published their flight data:

| Repository | Files | Licence |
|---|---|---|
| [NUSTARS/launch-vehicle](https://github.com/NUSTARS/launch-vehicle) | 46 | none stated |
| [dpfeldman/dpfeldman.github.io](https://github.com/dpfeldman/dpfeldman.github.io) | 10 | none stated |
| [RIT-Launch-Initiative/flight-data](https://github.com/RIT-Launch-Initiative/flight-data) | 10 | MIT |
| [osu-bsli/Airbrake-Control-Verification](https://github.com/osu-bsli/Airbrake-Control-Verification) | 7 | none stated |
| [burnoutlabs/post-flight-analysis](https://github.com/burnoutlabs/post-flight-analysis) | 6 | data CC BY 4.0 |
| [ISSUIUC/flight-data](https://github.com/ISSUIUC/flight-data) | 6 | none stated |
| [saividyud/NASA_25-26_Rocket_Simulations](https://github.com/saividyud/NASA_25-26_Rocket_Simulations) | 5 | none stated |
| [AerospaceNU/rocket-flight-data](https://github.com/AerospaceNU/rocket-flight-data) | 5 | GPL-3.0 |
| [legomyleg/BYU_Telemetry_HUD](https://github.com/legomyleg/BYU_Telemetry_HUD) | 4 | none stated |
| [Olin-Rocketry/kalman-filter-algorithm](https://github.com/Olin-Rocketry/kalman-filter-algorithm) | 4 | none stated |
| [njitrocketry/irec26-flight-data](https://github.com/njitrocketry/irec26-flight-data) | 3 | none stated |
| [njitrocketry/irec26-test-data](https://github.com/njitrocketry/irec26-test-data) | 3 | none stated |
| [WVU-Experimental-Rocketry-Club/Active_Drag_2024_2026](https://github.com/WVU-Experimental-Rocketry-Club/Active_Drag_2024_2026) | 3 | none stated |
| [icl-rocketry/art-data](https://github.com/icl-rocketry/art-data) | 2 | none stated |
| [Maeswoon/UNHSEDS](https://github.com/Maeswoon/UNHSEDS) | 2 | none stated |
| [matthewsimpsonaero/High-Powered-Rocketry](https://github.com/matthewsimpsonaero/High-Powered-Rocketry) | 2 | none stated |
| [NateOrlowski06/Rocket-Data](https://github.com/NateOrlowski06/Rocket-Data) | 2 | none stated |
| [Terrapin-Rocket-Team/Astra-Rocket](https://github.com/Terrapin-Rocket-Team/Astra-Rocket) | 2 | none stated |
| [AstroJays-Hopkins/Recovery-Software-Two-Stage](https://github.com/AstroJays-Hopkins/Recovery-Software-Two-Stage) | 1 | none stated |
| [cornellrocketryteam/6dof](https://github.com/cornellrocketryteam/6dof) | 1 | none stated |
| [Kaffos/MAGS_Simulation](https://github.com/Kaffos/MAGS_Simulation) | 1 | none stated |
| [osu-bsli/irec](https://github.com/osu-bsli/irec) | 1 | none stated |
| [psas/Launch-12](https://github.com/psas/Launch-12) | 1 | none stated |
| [rkuhlf/rocket-simulation](https://github.com/rkuhlf/rocket-simulation) | 1 | none stated |
| [Rose-Rocketry/2024-USLI-Vehicle](https://github.com/Rose-Rocketry/2024-USLI-Vehicle) | 1 | none stated |
| [usfsoar/NSL_25-26_EECS](https://github.com/usfsoar/NSL_25-26_EECS) | 1 | MIT |
| [usfsoar/SOAR_Simulacra](https://github.com/usfsoar/SOAR_Simulacra) | 1 | none stated |
| [victoria-field7/Student-Project-Soar-NASA-Student-Launch](https://github.com/victoria-field7/Student-Project-Soar-NASA-Student-Launch) | 1 | MIT |
| [wwang-8/L1_flight_data](https://github.com/wwang-8/L1_flight_data) | 1 | none stated |

If you own one of these logs and would rather it was not used, open an issue
and it will be removed from the manifest.

**Distinct flights.** Primary and backup computers in one rocket, and a Blue
Raven riding beside a TeleMega, are several logs of one flight. The 125
replayed logs cover about 86 distinct flights. The spec asks gate 3 for 100+.

RIT's 2023 KONG flight (two RRC3 logs, drogue at about 6 m/s) was held out
for a while: the pod confirmed the canopy about 1 s before the main fired.
The detector now handles it and both logs are back (docs/FEASIBILITY.md,
item 20).

**Not included:**
- Eggtimer logs (reader support exists). They drop to 2 Hz in descent, below
  the 10 Hz floor for this set.
- Blue Raven units whose main channel never fired (no deployment truth).
- Logs that are corrupted or have no barometer (EasyMotor, telemetry-rate
  CSVs).

More logs are welcome: add a line to `manifest.tsv` (name, repository, commit,
path, sha256, licence).
