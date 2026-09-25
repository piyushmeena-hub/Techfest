# UAV-X Stage 1 — Acceptance Report

**Generated:** 2026-09-25 19:56:05

**Overall Result:** ❌ SOME TESTS FAILED

## Mission Summary
| Metric | Value |
|--------|-------|
| total_pois | 7 |
| surveyed_pois | 6 |
| pending_pois | 0 |
| survivors_detected | 1 |
| survivor_pois | ['POI-C'] |
| packets_delivered | 6 |
| failed_uavs | ['UAV-05'] |
| total_events | 35 |

## Acceptance Tests
| Test | Result | Evidence |
|------|--------|----------|
| PoI Surveying | ✅ PASS | 6/7 PoIs surveyed (85%) |
| End-to-End GCS Delivery | ✅ PASS | 6 packets delivered to GCS out of 6 surveyed PoIs |
| Relay Reassignment | ✅ PASS | 1 RTL events triggered, 4 relay assignments made |
| Communication Recovery | ✅ PASS | 2 delivery failures recovered, 6 eventual deliveries |
| Priority Data Handling | ✅ PASS | Critical packets avg delivery tick: [48, 113], Low: [] |
| Battery-Aware RTL (No Crashes) | ✅ PASS | 1 RTL events triggered, 0 battery-crash failures |
| No UAV Crashes / Safety | ❌ FAIL | 1 UAV failures detected |
| Reproducible Logs | ✅ PASS | Event log: 35 entries. GCS log: 6 entries. Telemetry: Complete |

## Network Connectivity
| Metric | Value |
|--------|-------|
| total_nodes | 10 |
| reachable_to_gcs | 10 |
| isolated_nodes | [] |
| active_links | 21 |
| weak_links | 8 |
| down_links | 2 |