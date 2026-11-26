# Third-party flight logs in this folder

These real flight logs come from other teams' public repositories, unchanged.
Each is redistributed under its MIT licence; the notices below must stay with
the files.

| File | Source | Commit | Notes |
|---|---|---|---|
| `und-arc_irec2024_primary_telemega.csv` | [UND-ARC/IREC-ARC-2024](https://github.com/UND-ARC/IREC-ARC-2024) `Analysis/IREC_Primary_Flight_Data.csv` | `94a991b` | TeleMega, first 2 s of boost only |
| `und-arc_irec2024_secondary.csv` | same, `Analysis/IREC_Secondary_Flight_Data.csv` | `94a991b` | no accelerometer (EasyMini-style), 3.6 km apogee |
| `boulaetans_green-eggs.csv` | [boulaetans/numericalrocketry](https://github.com/boulaetans/numericalrocketry) `data/green_eggs_real_flight_raw.csv` | `136ab67` | small single-deploy flight, 116 m |

The larger public set (66 more logs, most with no licence) is not copied here:
`tools/fetch_public_logs.sh` downloads it, see [../public/README.md](../public/README.md).

## UND-ARC/IREC-ARC-2024

MIT License

Copyright (c) 2023 University of North Dakota Advanced Rocketry Club

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## boulaetans/numericalrocketry

MIT License

Copyright (c) 2026 Boula Etans

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
