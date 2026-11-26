# Real flight logs for gate 3

Drop host flight-computer CSV exports here (one flight per file). `grreplay`
picks up every `*.csv` in this folder on every CI run.

Accepted columns (header row, case-insensitive):

| column | meaning |
|---|---|
| `time` | seconds |
| `acceleration` or `accel` | m/s², along the airframe axis, gravity removed (AltOS convention: 0 on the pad) |
| `pressure` (Pa or hPa) **or** `height` / `altitude` (m) | barometer |
| `state_name` or `state` | host flight state; the first `main` row marks the main deployment |
| `main_t` (optional) | deployment time in seconds, if the log has no state column |

AltOS (TeleMetrum, EasyMega, ...) CSV exports from AltosUI work as-is. Other
brands: rename columns to match. Only add logs you have permission to share.
