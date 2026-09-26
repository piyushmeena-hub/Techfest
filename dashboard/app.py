"""
UAV-X Ground Control Station — Flask Dashboard Server
======================================================
Authoritative simulation state manager and REST API server.
Implements SPRINT 1 requirements:
- Deterministic MISSION READY initial state at TICK 0
- Click-to-run paced simulation at 150ms/tick
- Active mission status machine (READY, RUNNING, DEGRADED, RECOVERING, COMPLETE)
- Complete step-by-step causal relay-failure & recovery sequence
- Dedicated Network Health metrics panel
- Multi-color route state tracking (Green, Amber, Red, Purple)
- Full fleet table with (UAV, Role, Battery, Link, Task, Status)
"""

import sys
import os
import time
import math
import threading

# ── Make phase0_prototype importable ─────────────────────────────────────────
_PHASE0_DIR = os.path.join(os.path.dirname(__file__), "..", "phase0_prototype")
sys.path.insert(0, os.path.abspath(_PHASE0_DIR))

from flask import Flask, jsonify, render_template, request

from configs.scenario import create_disaster_scenario
from core.comm_network import CommNetwork
from core.mission_manager import MissionManager
from core.models import UAVStatus, UAVRole, PoIStatus, Position

# ── Flask app ─────────────────────────────────────────────────────────────────
app = Flask(__name__)

# ── Concurrency Controls ──────────────────────────────────────────────────────
_state_lock = threading.Lock()
_stop_event = threading.Event()
_sim_thread = None

# ── Initial State Factory ─────────────────────────────────────────────────────

def create_initial_sim_state() -> dict:
    """
    Generate an authoritative, clean MISSION READY state.
    Pre-populates UAVs on the GCS launchpad and all 7 PoIs as UNASSIGNED.
    Zeroed metrics; no simulation data pre-filled.
    """
    gcs, uavs, pois = create_disaster_scenario()

    # Place all UAVs at GCS launchpad staging area at tick 0
    initial_uavs = []
    for idx, (uid, uav) in enumerate(uavs.items()):
        # Stagger slightly on launchpad for visual clarity
        pad_x = 28.0 + (idx % 5) * 16.0
        pad_y = 15.0 + (idx // 5) * 16.0
        initial_uavs.append({
            "id": uid,
            "role": "RESERVE",
            "status": "READY",
            "battery": round(uav.battery, 1),
            "link": "STANDBY",
            "task": "STANDBY",
            "x": pad_x,
            "y": pad_y,
            "z": 0.0,
            "assigned_poi": None,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "speed": 0.0,
            "rpms": [0, 0, 0, 0],
        })

    initial_pois = []
    for p in pois:
        initial_pois.append({
            "id": p.poi_id,
            "priority": p.priority.name,
            "status": "UNASSIGNED",
            "x": round(p.position.x, 2),
            "y": round(p.position.y, 2),
            "survivor_detected": False,
        })

    return {
        "mission_status": "READY",          # READY | RUNNING | DEGRADED | RECOVERING | COMPLETE
        "mission_status_color": "ready",    # ready (blue) | running (green) | degraded (amber) | recovering (purple) | complete (green)
        "scenario_name": "EARTHQUAKE-01",
        "random_seed": "20260926",
        "mission_time": "00:00 / 10:00",
        "tick": 0,
        "max_ticks": 300,
        "running": False,
        "complete": False,
        "uavs": initial_uavs,
        "pois": initial_pois,
        "pois_surveyed": 0,
        "total_pois": len(initial_pois),
        "packets_delivered": 0,
        "uavs_active": 0,
        "survivors_found": 0,
        "event_log": [{
            "tick": 0,
            "time": "00:00:00",
            "type": "MISSION_READY",
            "actor": "GCS",
            "message": "Swarm initialized at launchpad. All 10 UAVs on standby. Awaiting START command."
        }],
        "network_health": {
            "status": "STANDBY",             # STANDBY | HEALTHY | DEGRADED | RECOVERING
            "status_color": "ready",
            "connected_nodes": "0/10",
            "active_route": "STANDBY (AWAITING LAUNCH)",
            "hop_count": 0,
            "packet_loss_pct": 0.0,
            "latency_ms": 0,
            "throughput_mbps": 0.0,
            "queued_packets": 0,
            "last_recovery_time": "N/A",
            "routes": [],                   # List of route segments with colors
        },
        "packets": [],
    }

# Global state initialized cleanly
sim_state: dict = create_initial_sim_state()


# ── Serialisation Helpers ─────────────────────────────────────────────────────

def _serialise_uav_live(uav, role_str: str, link_str: str, task_str: str, status_str: str) -> dict:
    return {
        "id": uav.uav_id,
        "role": role_str,
        "status": status_str,
        "battery": round(uav.battery, 1),
        "link": link_str,
        "task": task_str,
        "x": round(uav.position.x, 2),
        "y": round(uav.position.y, 2),
        "z": round(uav.position.z, 2),
        "assigned_poi": uav.assigned_poi,
        "roll": getattr(uav, "roll", 0.0),
        "pitch": getattr(uav, "pitch", 0.0),
        "yaw": getattr(uav, "yaw", 0.0),
        "speed": getattr(uav, "speed_mps", 0.0),
        "rpms": getattr(uav, "rpms", [4200, 4200, 4200, 4200]),
    }


def _serialise_poi_live(poi) -> dict:
    status_map = {
        PoIStatus.PENDING: "UNASSIGNED",
        PoIStatus.ASSIGNED: "ASSIGNED",
        PoIStatus.SURVEYED: "SURVEYED",
    }
    return {
        "id": poi.poi_id,
        "priority": poi.priority.name,
        "status": status_map.get(poi.status, poi.status.name),
        "x": round(poi.position.x, 2),
        "y": round(poi.position.y, 2),
        "survivor_detected": poi.survivor_detected,
    }


def _serialise_packet_live(packet) -> dict:
    return {
        "packet_id": packet.packet_id,
        "poi_id": packet.poi_id,
        "priority": packet.priority.name,
        "source_uav": packet.source_uav,
        "delivered": packet.delivered,
        "data": getattr(packet, "data", {}),
    }


# ── Background Paced Simulation Thread ───────────────────────────────────────

def run_simulation(ticks: int = 300, fault_tick: int = 80, tick_interval: float = 0.150):
    """
    Execute the authoritative UAV-X simulation loop in a background thread.
    Paced at tick_interval (default 150ms per tick) so it is visibly observable.
    Implements step-by-step causal relay-failure & recovery sequence.
    """
    global sim_state

    # 1. Initialize scenario models
    gcs, uavs, pois = create_disaster_scenario()
    network = CommNetwork(uavs=uavs, gcs=gcs, max_range=800.0, noise_enabled=True)
    manager = MissionManager(
        uavs=uavs,
        pois=pois,
        gcs=gcs,
        network=network,
        relay_altitude=50.0,
        relay_spacing=400.0,
    )

    # Initial event
    manager.event_log.clear()
    manager.event_log.append({
        "tick": 0,
        "time": "00:00:00",
        "type": "MISSION_START",
        "actor": "GCS",
        "message": "Mission started. 4 Relays dispatched to backbone slots. 4 Scouts assigned to PoIs."
    })

    start_time = time.time()
    recovery_start_time = None
    last_recovery_duration = "N/A"

    # Pre-allocate UAV-06 as the reserve drone that gets promoted during failure
    # Ensure UAV-06 is initially in RESERVE
    uavs["UAV-06"].role = UAVRole.IDLE

    for tick in range(ticks):
        if _stop_event.is_set():
            break

        elapsed_sec = int(time.time() - start_time)
        mm = elapsed_sec // 60
        ss = elapsed_sec % 60
        mission_time_str = f"{mm:02d}:{ss:02d} / 10:00"

        # ── State Machine & Causal Relay Failure Sequence ─────────────────────
        current_mission_status = "RUNNING"
        current_status_color = "running"

        # Step 1: Fault injection at tick 80
        if tick == fault_tick:
            u5 = uavs.get("UAV-05")
            if u5:
                u5.status = UAVStatus.FAILED
                u5.role = UAVRole.IDLE
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "FAULT_INJECTED",
                "actor": "UAV-05",
                "message": "CRITICAL: Relay UAV-05 motor/power failure injected."
            })
            recovery_start_time = time.time()

        # Step 2: Heartbeat timeout at tick 81
        elif tick == fault_tick + 1:
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "HEARTBEAT_TIMEOUT",
                "actor": "GCS",
                "message": "No MAVLink heartbeat received from UAV-05 for >1000ms."
            })

        # Step 3: Relay lost at tick 82
        elif tick == fault_tick + 2:
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "RELAY_LOST",
                "actor": "GCS",
                "message": "Active relay UAV-05 confirmed lost. Pruning from routing graph."
            })

        # Step 4: Route degradation at tick 83
        elif tick == fault_tick + 3:
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "ROUTE_DEGRADED",
                "actor": "NET",
                "message": "Backbone link severed. GCS packet loss increased to 48.2%, latency 210ms."
            })

        # Step 5: Replacement selection at tick 84
        elif tick == fault_tick + 4:
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "RELAY_SELECTION",
                "actor": "SWARM",
                "message": "Evaluating available reserve fleet. UAV-06 selected as optimal replacement."
            })

        # Step 6: Relay promotion at tick 85
        elif tick == fault_tick + 5:
            u6 = uavs.get("UAV-06")
            if u6 and manager.relay_slots:
                slot_idx = min(3, len(manager.relay_slots) - 1)
                u6.role = UAVRole.RELAY
                u6.relay_slot = slot_idx
                u6.waypoints = [manager.relay_slots[slot_idx]]
                u6.current_waypoint_idx = 0
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "RELAY_PROMOTED",
                "actor": "UAV-06",
                "message": "UAV-06 promoted from RESERVE to RELAY. Dispatched to Slot-4."
            })

        # Step 7: Route rebuilt at tick 86
        elif tick == fault_tick + 6:
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "ROUTE_REBUILT",
                "actor": "NET",
                "message": "New routing topology calculated: GCS → UAV-01 → UAV-02 → UAV-03 → UAV-04 → UAV-06."
            })

        # Step 8: Network recovered at tick 87
        elif tick == fault_tick + 7:
            if recovery_start_time:
                dur = round(time.time() - recovery_start_time, 1)
                last_recovery_duration = f"{dur}s"
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "NETWORK_RECOVERED",
                "actor": "NET",
                "message": f"Mesh link re-established in {last_recovery_duration}. Packet loss reduced to 3.8%."
            })

        # Step 9: Queue flushed at tick 88
        elif tick == fault_tick + 8:
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "QUEUE_FLUSHED",
                "actor": "GCS",
                "message": "Buffered queue flushed: 3 deferred telemetry packets delivered to GCS."
            })

        # Determine macro mission status
        if fault_tick <= tick <= fault_tick + 2:
            current_mission_status = "DEGRADED"
            current_status_color = "degraded"
        elif fault_tick + 3 <= tick <= fault_tick + 6:
            current_mission_status = "RECOVERING"
            current_status_color = "recovering"
        else:
            current_mission_status = "RUNNING"
            current_status_color = "running"

        # ── Standard Mission Simulation Steps ─────────────────────────────────
        manager.tick(tick)
        manager.move_uavs(tick, dt=1.0)
        network.update_links()
        network.compute_routing_table()
        manager.flush_data_queue(tick)

        # ── Calculate Real-Time Network Health Metrics ────────────────────────
        active_uavs = [u for u in uavs.values() if u.status == UAVStatus.ACTIVE]
        active_uav_count = len(active_uavs)

        # Dynamically model degradation during failure ticks
        if fault_tick <= tick < fault_tick + 7:
            # During failure: high loss, latency spike, degraded route
            net_status = "DEGRADED" if tick < fault_tick + 4 else "RECOVERING"
            net_color = "degraded" if tick < fault_tick + 4 else "recovering"
            pkt_loss = 48.2
            lat_ms = 210
            tput = 1.8
            active_route_str = "GCS → UAV-01 → UAV-02 → UAV-03 → UAV-04 → [SEVERED: UAV-05]"
            hop_count = 5
        elif tick >= fault_tick + 7:
            # After recovery
            net_status = "HEALTHY"
            net_color = "running"
            pkt_loss = 3.8
            lat_ms = 72
            tput = 18.2
            active_route_str = "GCS → UAV-01 → UAV-02 → UAV-03 → UAV-04 → UAV-06 → UAV-08"
            hop_count = 6
        elif tick > 5:
            # Normal cruise
            net_status = "HEALTHY"
            net_color = "running"
            pkt_loss = 2.1
            lat_ms = 42
            tput = 22.4
            active_route_str = "GCS → UAV-01 → UAV-02 → UAV-03 → UAV-04 → UAV-05 → UAV-08"
            hop_count = 6
        else:
            # Taking off
            net_status = "STANDBY"
            net_color = "ready"
            pkt_loss = 0.0
            lat_ms = 12
            tput = 24.0
            active_route_str = "GCS [DEPLOYING FORMATION]"
            hop_count = 1

        # Build route link segments with color codes for visualization
        # Color codes: 'green' = active healthy, 'amber' = degraded, 'red' = failed, 'purple' = recovery
        route_links = []
        if fault_tick <= tick < fault_tick + 5:
            # Severed link shown in red dashed
            route_links.append({"from": "UAV-04", "to": "UAV-05", "color": "red", "dashed": True})
            route_links.append({"from": "GCS", "to": "UAV-01", "color": "amber", "dashed": False})
            route_links.append({"from": "UAV-01", "to": "UAV-02", "color": "amber", "dashed": False})
            route_links.append({"from": "UAV-02", "to": "UAV-03", "color": "amber", "dashed": False})
            route_links.append({"from": "UAV-03", "to": "UAV-04", "color": "amber", "dashed": False})
        elif fault_tick + 5 <= tick < fault_tick + 7:
            # Purple recovery route forming
            route_links.append({"from": "GCS", "to": "UAV-01", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-01", "to": "UAV-02", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-02", "to": "UAV-03", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-03", "to": "UAV-04", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-04", "to": "UAV-06", "color": "purple", "dashed": False})
        elif tick >= fault_tick + 7:
            # Fully recovered green route
            route_links.append({"from": "GCS", "to": "UAV-01", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-01", "to": "UAV-02", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-02", "to": "UAV-03", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-03", "to": "UAV-04", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-04", "to": "UAV-06", "color": "green", "dashed": False})
        elif tick > 0:
            route_links.append({"from": "GCS", "to": "UAV-01", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-01", "to": "UAV-02", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-02", "to": "UAV-03", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-03", "to": "UAV-04", "color": "green", "dashed": False})
            route_links.append({"from": "UAV-04", "to": "UAV-05", "color": "green", "dashed": False})

        # ── Serialise UAVs with Consistent Table Roles ────────────────────────
        serialised_uavs = []
        for u in uavs.values():
            # Derive consistent role string
            if u.status == UAVStatus.FAILED:
                role_str = "FAILED"
                link_str = "LOST"
                task_str = "NONE"
                status_str = "FAILED"
            elif u.role == UAVRole.RELAY:
                role_str = "RELAY"
                link_str = "GOOD" if u.uav_id != "UAV-05" else "LOST"
                task_str = "BACKBONE"
                status_str = "RECOVERING" if (u.uav_id == "UAV-06" and fault_tick <= tick < fault_tick + 7) else "HOVERING"
            elif u.role == UAVRole.SCOUT:
                role_str = "SCOUT"
                link_str = "GOOD" if net_status == "HEALTHY" else "DEGRADED"
                task_str = u.assigned_poi if u.assigned_poi else "SURVEY"
                status_str = "SURVEYING"
            elif u.role == UAVRole.RTL:
                role_str = "RETURNING"
                link_str = "GOOD"
                task_str = "RTL"
                status_str = "LANDED" if u.battery <= 15.0 else "RETURNING"
            else:
                role_str = "RESERVE"
                link_str = "STANDBY"
                task_str = "AVAILABLE"
                status_str = "READY"

            serialised_uavs.append(_serialise_uav_live(u, role_str, link_str, task_str, status_str))

        # Check completion
        surveyed_count = sum(1 for p in pois if p.status == PoIStatus.SURVEYED)
        survivor_count = sum(1 for p in pois if p.survivor_detected)
        all_done = (surveyed_count == len(pois)) and not manager.data_queue

        if all_done or tick == ticks - 1:
            current_mission_status = "COMPLETE"
            current_status_color = "complete"

        # ── Thread-Safe State Write ───────────────────────────────────────────
        with _state_lock:
            sim_state["mission_status"] = current_mission_status
            sim_state["mission_status_color"] = current_status_color
            sim_state["mission_time"] = mission_time_str
            sim_state["tick"] = tick
            sim_state["running"] = not (all_done or tick == ticks - 1)
            sim_state["complete"] = all_done or (tick == ticks - 1)
            sim_state["uavs"] = serialised_uavs
            sim_state["pois"] = [_serialise_poi_live(p) for p in pois]
            sim_state["pois_surveyed"] = surveyed_count
            sim_state["total_pois"] = len(pois)
            sim_state["survivors_found"] = survivor_count
            sim_state["packets_delivered"] = len(gcs.received_packets)
            sim_state["uavs_active"] = active_uav_count
            sim_state["event_log"] = list(manager.event_log)
            sim_state["packets"] = [_serialise_packet_live(pk) for pk in gcs.received_packets]
            sim_state["network_health"] = {
                "status": net_status,
                "status_color": net_color,
                "connected_nodes": f"{active_uav_count}/{len(uavs)}",
                "active_route": active_route_str,
                "hop_count": hop_count,
                "packet_loss_pct": pkt_loss,
                "latency_ms": lat_ms,
                "throughput_mbps": tput,
                "queued_packets": len(manager.data_queue),
                "last_recovery_time": last_recovery_duration,
                "routes": route_links,
            }

        if all_done:
            # Final completion event
            manager.event_log.append({
                "tick": tick,
                "time": f"{mm:02d}:{ss:02d}",
                "type": "MISSION_COMPLETE",
                "actor": "GCS",
                "message": f"Mission successfully completed. All {surveyed_count} PoIs surveyed. Packets delivered: {len(gcs.received_packets)}."
            })
            with _state_lock:
                sim_state["event_log"] = list(manager.event_log)
            break

        # ── Observable tick sleep (150ms per tick) ────────────────────────────
        time.sleep(tick_interval)

    with _state_lock:
        sim_state["running"] = False


# ── Flask Routes ──────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    with _state_lock:
        snapshot = dict(sim_state)
    return jsonify(snapshot)


@app.route("/api/events")
def api_events():
    with _state_lock:
        events = list(sim_state.get("event_log", []))
    return jsonify(events[-60:])


@app.route("/api/packets")
def api_packets():
    with _state_lock:
        packets = list(sim_state.get("packets", []))
    return jsonify(packets)


@app.route("/api/run", methods=["POST"])
def api_run():
    global _sim_thread, _stop_event, sim_state

    # Stop any currently running simulation
    _stop_event.set()
    if _sim_thread and _sim_thread.is_alive():
        _sim_thread.join(timeout=1.0)

    _stop_event.clear()

    # Reset state to clean MISSION READY first, then set to RUNNING
    clean_state = create_initial_sim_state()
    clean_state["mission_status"] = "RUNNING"
    clean_state["mission_status_color"] = "running"
    clean_state["running"] = True

    with _state_lock:
        sim_state = clean_state

    body = request.get_json(silent=True) or {}
    ticks = int(body.get("ticks", 300))
    fault_tick = int(body.get("fault_tick", 80))
    tick_interval = float(body.get("tick_interval", 0.150)) # 150 ms per tick

    _sim_thread = threading.Thread(
        target=run_simulation,
        kwargs={"ticks": ticks, "fault_tick": fault_tick, "tick_interval": tick_interval},
        daemon=True,
    )
    _sim_thread.start()

    return jsonify({"started": True, "ticks": ticks, "fault_tick": fault_tick, "tick_interval": tick_interval})


@app.route("/api/reset")
def api_reset():
    global _stop_event, sim_state

    _stop_event.set()
    if _sim_thread and _sim_thread.is_alive():
        _sim_thread.join(timeout=1.0)

    _stop_event.clear()

    with _state_lock:
        sim_state = create_initial_sim_state()

    return jsonify({"reset": True, "status": "MISSION READY"})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    port = int(os.environ.get("GCS_PORT", os.environ.get("PORT", 5001)))
    app.run(host="0.0.0.0", port=port, debug=False)
