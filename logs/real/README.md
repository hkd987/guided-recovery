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
allows them a longer window. Other brands: rename columns to match.

Only add logs you have permission to share. The three logs here now are
MIT-licensed; see [THIRD_PARTY.md](THIRD_PARTY.md).
