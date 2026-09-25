# UAV-X Stage 1 — Acceptance Report

**Generated:** 2026-09-25 19:55:44

**Overall Result:** ❌ SOME TESTS FAILED

## Mission Summary
| Metric | Value |
|--------|-------|
| total_pois | 7 |
| surveyed_pois | 6 |
| pending_pois | 0 |
| survivors_detected | 1 |
| survivor_pois | ['POI-A'] |
| packets_delivered | 6 |
| failed_uavs | ['UAV-05', 'UAV-06', 'UAV-07', 'UAV-08', 'UAV-10'] |
| total_events | 39 |

## Acceptance Tests
| Test | Result | Evidence |
|------|--------|----------|
| PoI Surveying | ✅ PASS | 6/7 PoIs surveyed (85%) |
| End-to-End GCS Delivery | ✅ PASS | 6 packets delivered to GCS out of 6 surveyed PoIs |
| Relay Reassignment | ✅ PASS | 5 RTL events triggered, 4 relay assignments made |
| Communication Recovery | ✅ PASS | 2 delivery failures recovered, 6 eventual deliveries |
| Priority Data Handling | ✅ PASS | Critical packets avg delivery tick: [48, 113], Low: [171] |
| Battery-Aware RTL (No Crashes) | ❌ FAIL | 5 RTL events triggered, 4 battery-crash failures |
| No UAV Crashes / Safety | ❌ FAIL | 5 UAV failures detected |
| Reproducible Logs | ✅ PASS | Event log: 39 entries. GCS log: 6 entries. Telemetry: Complete |

## Network Connectivity
| Metric | Value |
|--------|-------|
| total_nodes | 10 |
| reachable_to_gcs | 10 |
| isolated_nodes | [] |
| active_links | 26 |
| weak_links | 11 |
| down_links | 3 |