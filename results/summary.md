# Results (seed 0, 16200 trials)

Lateral tolerance from fork/pocket geometry: +/-30.0 mm. Error columns are 90th percentiles of ground truth: lateral mm / angle deg.

## Ideal simulator vs less ideal ones

| Simulator | Controller | Loaded | Rate (95 % CI) | Error at approach target | Error at fork entry | Mean time (s) | Failures |
|---|---|---|---|---|---|---|---|
| ideal sim | Baseline | 500/500 | 100.0 % (99.2-100.0) | 15.4 / 0.56 | 0.2 / 0.03 | 26 | - |
| assumed real truck (1x) | Baseline | 464/500 | 92.8 % (90.2-94.8) | 145.8 / 0.85 | 23.0 / 2.33 | 29 | collision fork face 23, collision fork wall 13 |
| stressed (1.5x) | Baseline | 343/500 | 68.6 % (64.4-72.5) | 229.8 / 1.32 | 32.6 / 6.65 | 36 | collision fork face 98, collision fork wall 51, lost pallet 7, approach error too large 1 |
| ideal sim | Actuator-aware | 500/500 | 100.0 % (99.2-100.0) | 15.6 / 0.64 | 0.3 / 0.05 | 29 | - |
| assumed real truck (1x) | Actuator-aware | 463/500 | 92.6 % (90.0-94.6) | 59.9 / 0.92 | 23.9 / 0.21 | 38 | collision fork face 23, collision fork wall 14 |
| stressed (1.5x) | Actuator-aware | 360/500 | 72.0 % (67.9-75.8) | 93.4 / 1.31 | 35.6 / 0.36 | 49 | collision fork face 93, collision fork wall 40, lost pallet 6, timeout 1 |

## One error source at a time (rest at 1.5x nominal)

| Error source | Level | Baseline | Actuator-aware |
|---|---|---|---|
| (none changed) | - | 68.6 % | 72.0 % |
| lidar range noise | removed | 72.7 % | 75.3 % |
| lidar range noise | doubled | 2.7 % | 3.3 % |
| lidar dropout | removed | 68.7 % | 72.0 % |
| lidar dropout | doubled | 66.0 % | 69.3 % |
| lidar mount yaw error | removed | 98.0 % | 98.0 % |
| lidar mount yaw error | doubled | 42.7 % | 46.0 % |
| lidar mount lateral error | removed | 72.0 % | 74.0 % |
| lidar mount lateral error | doubled | 66.0 % | 70.0 % |
| scan/pose time offset | removed | 74.0 % | 72.7 % |
| scan/pose time offset | doubled | 51.3 % | 72.7 % |
| steering zero offset | removed | 74.0 % | 76.7 % |
| steering zero offset | doubled | 61.3 % | 68.7 % |
| steering lag | removed | 69.3 % | 74.0 % |
| steering lag | doubled | 31.3 % | 70.0 % |
| steering rate limit | removed | 74.0 % | 72.7 % |
| steering rate limit | doubled | 26.0 % | 70.0 % |
| command latency | removed | 74.0 % | 74.0 % |
| command latency | doubled | 43.3 % | 72.7 % |
| drive lag | removed | 54.0 % | 72.0 % |
| drive lag | doubled | 26.7 % | 70.7 % |
| wheel scale error | removed | 70.7 % | 72.0 % |
| wheel scale error | doubled | 68.0 % | 74.0 % |
| floor slip | removed | 72.7 % | 75.3 % |
| floor slip | doubled | 63.3 % | 68.0 % |

## Detection alone (single scans)

| Noise | Range (m) | Pallet found | Found but turned 90 deg | lateral p90 (mm) | angle p90 (deg) |
|---|---|---|---|---|---|
| 1x | 2 | 100.0 % | 0.0 % | 36.4 | 1.28 |
| 1x | 3 | 100.0 % | 0.0 % | 31.6 | 1.41 |
| 1x | 4 | 100.0 % | 0.0 % | 28.0 | 1.50 |
| 1x | 5 | 100.0 % | 0.0 % | 26.3 | 1.43 |
| 1x | 6 | 100.0 % | 0.0 % | 25.6 | 1.68 |
| 1x | 7 | 100.0 % | 0.0 % | 22.0 | 1.45 |
| 1x | 8 | 99.7 % | 0.0 % | 25.4 | 1.79 |
| 2x | 2 | 100.0 % | 0.0 % | 48.1 | 2.70 |
| 2x | 3 | 99.7 % | 0.3 % | 42.6 | 2.85 |
| 2x | 4 | 99.0 % | 1.0 % | 44.0 | 3.25 |
| 2x | 5 | 100.0 % | 0.0 % | 44.8 | 3.41 |
| 2x | 6 | 99.3 % | 0.3 % | 36.1 | 2.93 |
| 2x | 7 | 99.0 % | 0.3 % | 36.1 | 2.82 |
| 2x | 8 | 98.7 % | 1.3 % | 46.3 | 3.68 |
