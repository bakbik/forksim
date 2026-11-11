# Results (seed 0, 2610 trials)

Lateral tolerance from fork/pocket geometry: +/-30.0 mm. Error columns are 90th percentiles of ground truth: lateral mm / angle deg.

## Ideal simulator vs less ideal ones

| Simulator | Controller | Loaded | Rate (95 % CI) | Error at approach target | Error at fork entry | Mean time (s) | Failures |
|---|---|---|---|---|---|---|---|
| ideal sim | Baseline | 80/80 | 100.0 % (95.4-100.0) | 15.1 / 0.51 | 0.2 / 0.03 | 25 | - |
| assumed real truck (1x) | Baseline | 75/80 | 93.8 % (86.2-97.3) | 139.2 / 0.80 | 22.6 / 2.15 | 29 | collision fork face 5 |
| stressed (1.5x) | Baseline | 30/80 | 37.5 % (27.7-48.5) | 549.9 / 1.45 | 33.1 / 12.60 | 46 | collision fork face 25, collision fork wall 9, approach error too large 8, timeout 7, lost pallet 1 |
| ideal sim | Actuator-aware | 80/80 | 100.0 % (95.4-100.0) | 15.4 / 0.53 | 0.3 / 0.04 | 29 | - |
| assumed real truck (1x) | Actuator-aware | 73/80 | 91.2 % (83.0-95.7) | 63.8 / 0.91 | 24.2 / 0.19 | 39 | collision fork face 5, collision fork wall 2 |
| stressed (1.5x) | Actuator-aware | 56/80 | 70.0 % (59.2-78.9) | 102.7 / 1.32 | 36.2 / 0.28 | 61 | collision fork face 14, collision fork wall 8, timeout 2 |

## One error source at a time (rest at 1.5x nominal)

| Error source | Level | Baseline | Actuator-aware |
|---|---|---|---|
| (none changed) | - | 37.5 % | 70.0 % |
| lidar noise at the sensor | removed | 46.7 % | 73.3 % |
| lidar noise at the sensor | doubled | 0.0 % | 0.0 % |
| lidar noise growth with range | removed | 33.3 % | 86.7 % |
| lidar noise growth with range | doubled | 0.0 % | 0.0 % |
| lidar dropout | removed | 26.7 % | 80.0 % |
| lidar dropout | doubled | 33.3 % | 80.0 % |
| lidar mount yaw error | removed | 60.0 % | 100.0 % |
| lidar mount yaw error | doubled | 20.0 % | 26.7 % |
| lidar mount lateral error | removed | 40.0 % | 73.3 % |
| lidar mount lateral error | doubled | 26.7 % | 66.7 % |
| scan/pose time offset | removed | 26.7 % | 66.7 % |
| scan/pose time offset | doubled | 13.3 % | 80.0 % |
| steering zero offset | removed | 33.3 % | 80.0 % |
| steering zero offset | doubled | 26.7 % | 80.0 % |
| steering lag | removed | 26.7 % | 60.0 % |
| steering lag | doubled | 6.7 % | 80.0 % |
| steering rate limit | removed | 93.3 % | 86.7 % |
| steering rate limit | doubled | 13.3 % | 26.7 % |
| command latency | removed | 46.7 % | 86.7 % |
| command latency | doubled | 13.3 % | 86.7 % |
| drive lag | removed | 40.0 % | 73.3 % |
| drive lag | doubled | 13.3 % | 80.0 % |

## Approach distance D

| D (m) | Baseline 1x | Baseline 1.5x | Actuator-aware 1x | Actuator-aware 1.5x |
|---|---|---|---|---|
| 0.3 | 20.0 % (rejected 3/15) | 13.3 % (rejected 0/15) | 60.0 % (rejected 6/15) | 6.7 % (rejected 13/15) |
| 0.5 | 33.3 % (rejected 9/15) | 13.3 % (rejected 1/15) | 100.0 % (rejected 0/15) | 46.7 % (rejected 2/15) |
| 1 | 86.7 % (rejected 1/15) | 20.0 % (rejected 7/15) | 100.0 % (rejected 0/15) | 80.0 % (rejected 0/15) |
| 1.5 | 100.0 % (rejected 0/15) | 33.3 % (rejected 2/15) | 100.0 % (rejected 0/15) | 80.0 % (rejected 0/15) |
| 2 | 100.0 % (rejected 0/15) | 26.7 % (rejected 0/15) | 100.0 % (rejected 0/15) | 33.3 % (rejected 0/15) |
| 3 | 100.0 % (rejected 0/15) | 0.0 % (rejected 0/15) | 100.0 % (rejected 0/15) | 0.0 % (rejected 0/15) |

## Lag between perception and control (rest at 1x)

| Lag (ms) | Baseline | Actuator-aware | Baseline failures |
|---|---|---|---|
| 0 | 19/20 = 95 % | 19/20 = 95 % | collision fork face 1 |
| 160 | 19/20 = 95 % | 19/20 = 95 % | collision fork face 1 |
| 320 | 19/20 = 95 % | 19/20 = 95 % | collision fork face 1 |
| 480 | 11/20 = 55 % | 19/20 = 95 % | collision fork face 4, collision fork wall 4, approach error too large 1 |
| 640 | 3/20 = 15 % | 19/20 = 95 % | collision fork face 9, collision fork wall 7, approach error too large 1 |

## Start layout

| Facing at start | Errors | Baseline | Actuator-aware | Mean time (s) |
|---|---|---|---|---|
| entry edge ahead | 0x | 15/15 = 100.0 % | 15/15 = 100.0 % | 26 / 29 |
| entry edge ahead | 1x | 15/15 = 100.0 % | 15/15 = 100.0 % | 29 / 40 |
| entry edge ahead | 1.5x | 5/15 = 33.3 % | 12/15 = 80.0 % | 49 / 59 |
| corner ahead | 0x | 15/15 = 100.0 % | 15/15 = 100.0 % | 31 / 35 |
| corner ahead | 1x | 11/15 = 73.3 % | 15/15 = 100.0 % | 34 / 46 |
| corner ahead | 1.5x | 1/15 = 6.7 % | 9/15 = 60.0 % | 55 / 72 |
| side edge ahead | 0x | 15/15 = 100.0 % | 15/15 = 100.0 % | 35 / 40 |
| side edge ahead | 1x | 11/15 = 73.3 % | 15/15 = 100.0 % | 40 / 49 |
| side edge ahead | 1.5x | 0/15 = 0.0 % | 13/15 = 86.7 % | 60 / 74 |

## Detection alone (single scans)

| Noise | Range (m) | Pallet found | Found but turned 90 deg | lateral p90 head on (mm) | angle p90 head on (deg) | lateral p90 any view (mm) |
|---|---|---|---|---|---|---|
| 1x | 1 | 100.0 % | 0.0 % | 8.9 | 0.44 | 17.3 |
| 1x | 2 | 99.0 % | 1.0 % | 11.3 | 0.79 | 20.1 |
| 1x | 3 | 100.0 % | 0.0 % | 19.7 | 1.52 | 24.5 |
| 1x | 4 | 100.0 % | 0.0 % | 19.5 | 1.78 | 30.2 |
| 1x | 5 | 99.0 % | 1.0 % | 24.2 | 1.96 | 23.2 |
| 1x | 6 | 96.0 % | 1.0 % | 33.7 | 3.08 | 37.2 |
| 1x | 7 | 95.0 % | 1.0 % | 31.2 | 3.19 | 38.0 |
| 1x | 8 | 96.0 % | 1.0 % | 45.0 | 3.33 | 40.6 |
| 2x | 1 | 98.0 % | 0.0 % | 19.1 | 1.67 | 54.1 |
| 2x | 2 | 100.0 % | 0.0 % | 46.1 | 4.33 | 46.1 |
| 2x | 3 | 100.0 % | 0.0 % | 34.1 | 3.20 | 38.8 |
| 2x | 4 | 94.0 % | 0.0 % | 60.1 | 5.38 | 60.1 |
| 2x | 5 | 92.0 % | 0.0 % | 42.5 | 4.34 | 42.5 |
| 2x | 6 | 91.0 % | 0.0 % | 67.8 | 3.75 | 50.0 |
| 2x | 7 | 86.0 % | 5.0 % | 189.4 | 6.46 | 63.4 |
| 2x | 8 | 88.0 % | 0.0 % | 143.2 | 6.67 | 102.1 |
