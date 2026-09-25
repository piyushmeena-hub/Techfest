"""
UAV-X Ground Control Station — Flask Dashboard Server
======================================================
Provides a live web dashboard for the UAV-X Phase 0 simulation.

Routes
------
GET  /            → Render the GCS dashboard HTML page
GET  /api/status  → Full simulation state as JSON
GET  /api/events  → Last 50 event log entries as JSON
GET  /api/packets → GCS received packets as JSON
POST /api/run     → Start background simulation thread
GET  /api/reset   → Reset simulation state

The simulation runs in a background daemon thread, updating the
global ``sim_state`` dict every tick so the dashboard can poll it
without blocking the Flask request loop.
"""

import sys
import os
import time
import threading

# ── Make phase0_prototype importable ─────────────────────────────────────────
_PHASE0_DIR = os.path.join(os.path.dirname(__file__), "..", "phase0_prototype")
sys.path.insert(0, os.path.abspath(_PHASE0_DIR))

from flask import Flask, jsonify, render_template, request

from configs.scenario import create_disaster_scenario
from core.comm_network import CommNetwork
from core.mission_manager import MissionManager
from core.acceptance_reporter import AcceptanceReporter
from core.models import UAVStatus, UAVRole, PoIStatus

# ── Flask app ─────────────────────────────────────────────────────────────────
app = Flask(__name__)

# ── Global simulation state ───────────────────────────────────────────────────
# All values are kept JSON-serialisable so /api/status can dump them directly.
sim_state: dict = {
    "uavs": [],            # list of serialised UAV dicts
    "pois": [],            # list of serialised PoI dicts
    "network": {},         # connectivity report snapshot
    "event_log": [],       # full event log
    "packets": [],         # GCS received packet records
    "tick": 0,             # current simulation tick
    "running": False,      # True while background thread is active
    "complete": False,     # True once simulation finishes
    "survivors_found": 0,  # count of PoIs with survivor_detected=True
    "pois_surveyed": 0,    # count of surveyed PoIs
    "packets_delivered": 0,# count of packets successfully delivered to GCS
    "uavs_active": 0,      # UAVs currently ACTIVE (not landed/failed)
}

# Lock for thread-safe writes
_state_lock = threading.Lock()


# ── Serialisation helpers ─────────────────────────────────────────────────────

def _serialise_uav(uav) -> dict:
    """
    Convert a UAV dataclass instance into a JSON-serialisable dict.

    Parameters
    ----------
    uav : UAV
        The UAV object from the simulation.

    Returns
    -------
    dict
        Fields: id, role, status, battery, x, y, z, assigned_poi.
    """
    return {
        "id": uav.uav_id,
        "role": uav.role.value,
        "status": uav.status.value,
        "battery": round(uav.battery, 1),
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


def _serialise_poi(poi) -> dict:
    """
    Convert a PointOfInterest dataclass instance into a JSON-serialisable dict.

    Parameters
    ----------
    poi : PointOfInterest
        The PoI object from the simulation.

    Returns
    -------
    dict
        Fields: id, priority, status, x, y, survivor_detected.
    """
    return {
        "id": poi.poi_id,
        "priority": poi.priority.name,
        "status": poi.status.name,
        "x": round(poi.position.x, 2),
        "y": round(poi.position.y, 2),
        "survivor_detected": poi.survivor_detected,
    }


def _serialise_packet(packet) -> dict:
    """
    Convert a DataPacket dataclass instance into a JSON-serialisable dict.

    Parameters
    ----------
    packet : DataPacket
        The packet object stored in gcs.received_packets.

    Returns
    -------
    dict
        Fields: packet_id, poi_id, priority, source_uav, delivered.
    """
    return {
        "packet_id": packet.packet_id,
        "poi_id": packet.poi_id,
        "priority": packet.priority.name,
        "source_uav": packet.source_uav,
        "delivered": packet.delivered,
        "data": getattr(packet, "data", {}),
    }


# ── Background simulation thread ──────────────────────────────────────────────

def run_simulation(ticks: int = 300, fault_tick: int = 80):
    """
    Execute the full UAV-X simulation loop in a background thread.

    Mirrors the logic in phase0_prototype/main.py but runs headless
    (no terminal visualisation, no sleep delays) and updates the global
    ``sim_state`` dict every tick so the Flask API can serve live data.

    Parameters
    ----------
    ticks : int
        Maximum number of simulation ticks to run.
    fault_tick : int
        Tick number at which to inject a relay fault (UAV-05 killed).
        Set to 0 to disable fault injection.
    """
    global sim_state

    # ── Initialise scenario ───────────────────────────────────────────
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

    fault_injected = False

    # ── Main loop ─────────────────────────────────────────────────────
    for tick in range(ticks):

        # ① Fault injection — kill UAV-05 (a relay) at specified tick
        if fault_tick > 0 and tick == fault_tick and not fault_injected:
            target = uavs.get("UAV-05")
            if target:
                target.status = UAVStatus.FAILED
                target.role = UAVRole.IDLE
                manager.event_log.append({
                    "tick": tick,
                    "type": "FAULT_INJECTED",
                    "actor": "UAV-05",
                    "message": "Relay UAV-05 intentionally killed — testing recovery",
                })
            fault_injected = True

        # ② Mission manager tick (fault detection, RTL, relay & scout assignment)
        manager.tick(tick)

        # ③ Move UAVs toward their waypoints
        manager.move_uavs(tick, dt=1.0)

        # ④ Update communication network links and routing table
        network.update_links()
        network.compute_routing_table()

        # ⑤ Flush data queue — deliver packets to GCS via network
        manager.flush_data_queue(tick)

        # ⑥ Snapshot current state into sim_state (thread-safe)
        connectivity = network.get_connectivity_report()
        active_uav_count = sum(
            1 for u in uavs.values() if u.status == UAVStatus.ACTIVE
        )
        surveyed_count = sum(
            1 for p in pois if p.status == PoIStatus.SURVEYED
        )
        survivor_count = sum(
            1 for p in pois if p.survivor_detected
        )

        with _state_lock:
            sim_state["tick"] = tick
            sim_state["running"] = True
            sim_state["complete"] = False
            sim_state["uavs"] = [_serialise_uav(u) for u in uavs.values()]
            sim_state["pois"] = [_serialise_poi(p) for p in pois]
            sim_state["network"] = connectivity
            sim_state["event_log"] = list(manager.event_log)
            sim_state["packets"] = [_serialise_packet(pk) for pk in gcs.received_packets]
            sim_state["pois_surveyed"] = surveyed_count
            sim_state["survivors_found"] = survivor_count
            sim_state["packets_delivered"] = len(gcs.received_packets)
            sim_state["uavs_active"] = active_uav_count

        # ⑦ Early exit if all PoIs are surveyed and the data queue is empty
        all_done = all(p.status == PoIStatus.SURVEYED for p in pois)
        if all_done and not manager.data_queue:
            break

        # Small sleep so the thread doesn't pin the CPU
        time.sleep(0.03)

    # ── Simulation finished ───────────────────────────────────────────
    with _state_lock:
        sim_state["running"] = False
        sim_state["complete"] = True


# ── Flask Routes ──────────────────────────────────────────────────────────────

@app.route("/")
def index():
    """
    Serve the GCS dashboard HTML page.

    Returns
    -------
    flask.Response
        Rendered ``index.html`` template from the ``templates/`` directory.
    """
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    """
    Return the full current simulation state as JSON.

    The payload contains serialisable UAV and PoI objects, network
    connectivity stats, tick number, and running/complete flags.

    Returns
    -------
    flask.Response
        JSON object matching the ``sim_state`` dict structure.
    """
    with _state_lock:
        snapshot = dict(sim_state)
    return jsonify(snapshot)


@app.route("/api/events")
def api_events():
    """
    Return the last 50 simulation event log entries as JSON.

    Events are produced by the MissionManager and include types such as
    RELAY_ASSIGN, SCOUT_ASSIGN, SURVEY_COMPLETE, DELIVERED, RTL, FAULT, etc.

    Returns
    -------
    flask.Response
        JSON array of event dicts, most recent last.
    """
    with _state_lock:
        events = list(sim_state.get("event_log", []))
    return jsonify(events[-50:])


@app.route("/api/packets")
def api_packets():
    """
    Return all packets received by the GCS as JSON.

    Returns
    -------
    flask.Response
        JSON array of packet dicts (packet_id, poi_id, priority, source_uav).
    """
    with _state_lock:
        packets = list(sim_state.get("packets", []))
    return jsonify(packets)


@app.route("/api/run", methods=["POST"])
def api_run():
    """
    Start the background simulation thread if not already running.

    Accepts optional JSON body parameters:
      - ``ticks`` (int, default 300): number of simulation ticks
      - ``fault_tick`` (int, default 80): tick at which to inject relay fault

    Returns
    -------
    flask.Response
        JSON ``{"started": true}`` if the thread was launched,
        or ``{"started": false, "reason": "..."}`` if already running.
    """
    with _state_lock:
        already_running = sim_state.get("running", False)

    if already_running:
        return jsonify({"started": False, "reason": "Simulation already running"})

    body = request.get_json(silent=True) or {}
    ticks = int(body.get("ticks", 300))
    fault_tick = int(body.get("fault_tick", 80))

    thread = threading.Thread(
        target=run_simulation,
        kwargs={"ticks": ticks, "fault_tick": fault_tick},
        daemon=True,
    )
    thread.start()

    return jsonify({"started": True, "ticks": ticks, "fault_tick": fault_tick})


@app.route("/api/reset")
def api_reset():
    """
    Reset the global simulation state to its initial blank values.

    After calling this endpoint the dashboard will show an empty map and
    a new simulation can be started via POST /api/run.

    Returns
    -------
    flask.Response
        JSON ``{"reset": true}``.
    """
    with _state_lock:
        sim_state["uavs"] = []
        sim_state["pois"] = []
        sim_state["network"] = {}
        sim_state["event_log"] = []
        sim_state["packets"] = []
        sim_state["tick"] = 0
        sim_state["running"] = False
        sim_state["complete"] = False
        sim_state["survivors_found"] = 0
        sim_state["pois_surveyed"] = 0
        sim_state["packets_delivered"] = 0
        sim_state["uavs_active"] = 0
    return jsonify({"reset": True})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    port = int(os.environ.get("GCS_PORT", os.environ.get("PORT", 5001)))
    app.run(host="0.0.0.0", port=port, debug=False)

