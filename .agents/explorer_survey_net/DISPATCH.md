# Dispatch: Explorer 3 — FANET & Multi-Hop Network Architect

## Assignment
Investigate and design the resilient Flying Ad-Hoc Network (FANET) communication and multi-hop routing subsystem.

## Inputs
- ORIGINAL_REQUEST: `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`

## Objectives
1. Define wireless propagation and physical layer models:
   - Transmission range limits, path loss models (Friis/log-distance), SNR, and line-of-sight obstruction by buildings/terrain.
   - Dynamic link establishment and link quality metric (RSSI, packet error rate, delay).
2. Design multi-hop routing protocol:
   - Resilient routing algorithm (e.g. dynamic AODV, OLSR, or link-state shortest path routing with hop count and quality metrics).
   - Dynamic topology updates as UAVs move in 3D space.
   - Multi-hop packet routing and tracing: route discovery, packet header structure (src, dst, hops, payload, timestamp), packet relaying through intermediate UAVs to GCS.
   - Resilience against link disruption: route recalculation, alternate path failover, buffer-and-forward when links break.
3. Define logging and verification metrics:
   - Proving multi-hop communication (e.g. logging `UAV_3 -> UAV_1 -> GCS`, end-to-end latency, packet delivery ratio, hop distribution).
4. Write your comprehensive report to `d:\drone model\IIT Bombay\.agents\explorer_survey_net\survey_net_report.md` and deliver `handoff.md`.

## 2026-09-25T14:26:00Z
Received dispatch from parent orchestrator (id: 4ad727ae-0330-41e2-8017-656bde75909d).
Assigned to conduct authoritative technical survey on resilient multi-hop aerial communication networks (FANET).
Target deliverables:
1. `survey_net_report.md`
2. `handoff.md`
3. `progress.md`
4. Notify orchestrator via `send_message`.
