# Public real flight logs (fetched, not stored)

`manifest.tsv` lists 66 real flight logs that university and hobby rocketry
teams have posted on GitHub: AltosUI exports from TeleMega, TeleMetrum,
EasyMega and EasyMini flight computers, 40 m to 8.3 km apogee, single and dual
deploy, including failed and late mains. Gate 3 replays every one of them in
CI.

Most of these repositories state no licence, so the files are **not copied
into this repository**. `tools/fetch_public_logs.sh` downloads them at test
time from the exact commit listed, and checks each file's sha256, so every run
replays the same bytes:

```sh
tools/fetch_public_logs.sh                 # -> build/public-logs/
./build/grreplay --main-alt log --min-flights 50 logs/real/*.csv build/public-logs/*.csv
```

Thank you to the teams who published their flight data:

| Repository | Logs | Licence |
|---|---|---|
| [NUSTARS/launch-vehicle](https://github.com/NUSTARS/launch-vehicle) | 37 | none stated |
| [dpfeldman/dpfeldman.github.io](https://github.com/dpfeldman/dpfeldman.github.io) | 10 | none stated |
| [ISSUIUC/flight-data](https://github.com/ISSUIUC/flight-data) | 6 | none stated |
| [saividyud/NASA_25-26_Rocket_Simulations](https://github.com/saividyud/NASA_25-26_Rocket_Simulations) | 5 | none stated |
| [burnoutlabs/post-flight-analysis](https://github.com/burnoutlabs/post-flight-analysis) | 4 | data CC BY 4.0 |
| [AstroJays-Hopkins/Recovery-Software-Two-Stage](https://github.com/AstroJays-Hopkins/Recovery-Software-Two-Stage) | 1 | none stated |
| [NateOrlowski06/Rocket-Data](https://github.com/NateOrlowski06/Rocket-Data) | 1 | none stated |
| [Rose-Rocketry/2024-USLI-Vehicle](https://github.com/Rose-Rocketry/2024-USLI-Vehicle) | 1 | none stated |
| [wwang-8/L1_flight_data](https://github.com/wwang-8/L1_flight_data) | 1 | none stated |

If you own one of these logs and would rather it was not used, open an issue
and it will be removed from the manifest.

Primary and backup computers in the same rocket count as two logs of one
flight: the 66 logs plus the 3 in `logs/real/` cover about 50 distinct
flights. The spec asks gate 3 for 100+ real flights, so more logs are welcome:
add a line to `manifest.tsv` (name, repository, commit, path, sha256, licence).
