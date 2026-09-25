"""
UAV-X Phase 0: Main Simulation Entry Point
Run the full post-disaster UAV swarm mission simulation.

Usage:
    python main.py              # run with live visualization
    python main.py --no-viz     # run headless (faster)
    python main.py --ticks 300  # set number of simulation ticks
"""
import sys
import os
import time
import argparse

# Add parent dir to path for imports
sys.path.insert(0, os.path.dirname(__file__))

from core.models import Position
from core.comm_network import CommNetwork
from core.mission_manager import MissionManager
from core.acceptance_reporter import AcceptanceReporter
from configs.scenario import create_disaster_scenario
from visualizer.terminal_viz import render_frame


def parse_args():
    parser = argparse.ArgumentParser(description="UAV-X Disaster Response Simulation")
    parser.add_argument("--ticks", type=int, default=200, help="Number of simulation ticks")
    parser.add_argument("--no-viz", action="store_true", help="Run headless (no terminal display)")
    parser.add_argument("--tick-delay", type=float, default=0.05, help="Seconds between ticks")
    parser.add_argument("--fault-tick", type=int, default=80,
                        help="Tick at which to inject a relay UAV fault (0=disabled)")
    parser.add_argument("--no-report", action="store_true", help="Skip acceptance report")
    return parser.parse_args()


def main():
    args = parse_args()

    print("═" * 60)
    print("  UAV-X: POST-DISASTER AERIAL SWARM SYSTEM")
    print("  Phase 0 — Algorithm Prototype")
    print("═" * 60)
    print(f"\n  Ticks: {args.ticks} | Viz: {not args.no_viz} | Fault at tick: {args.fault_tick}")
    print("\n  Initialising scenario...")

    # ── Setup ─────────────────────────────────────────────
    gcs, uavs, pois = create_disaster_scenario()

    network = CommNetwork(uavs=uavs, gcs=gcs, max_range=800.0, noise_enabled=True)

    manager = MissionManager(
        uavs=uavs, pois=pois, gcs=gcs, network=network,
        relay_altitude=50.0, relay_spacing=400.0
    )

    reporter = AcceptanceReporter(manager=manager, network=network)

    print(f"  GCS at: {gcs.position}")
    print(f"  UAVs:   {len(uavs)}")
    print(f"  PoIs:   {len(pois)}")
    print(f"  Relay slots: {len(manager.relay_slots)}")
    print("\n  Starting simulation...\n")
    time.sleep(1.0)

    # ── Simulation Loop ───────────────────────────────────
    fault_injected = False

    for tick in range(args.ticks):

        # ① Fault injection — kill UAV-05 (a relay) at specified tick
        if args.fault_tick > 0 and tick == args.fault_tick and not fault_injected:
            from core.models import UAVStatus, UAVRole
            target = uavs.get("UAV-05")
            if target:
                target.status = UAVStatus.FAILED
                target.role = UAVRole.IDLE
                manager.event_log.append({
                    "tick": tick, "type": "FAULT_INJECTED",
                    "actor": "UAV-05",
                    "message": "Relay UAV-05 intentionally killed — testing recovery"
                })
                print(f"\n  ⚡ FAULT INJECTED at tick {tick}: UAV-05 killed (relay failure test)")
            fault_injected = True

        # ② Run mission manager tick
        manager.tick(tick)

        # ③ Move UAVs toward waypoints
        manager.move_uavs(tick, dt=1.0)

        # ④ Update communication network
        network.update_links()
        network.compute_routing_table()

        # ⑤ Flush data queue → deliver packets to GCS
        manager.flush_data_queue(tick)

        # ⑥ Visualize
        if not args.no_viz and tick % 3 == 0:
            os.system("clear" if os.name == "posix" else "cls")
            frame = render_frame(tick, uavs, pois, gcs, network)
            print(frame)
            time.sleep(args.tick_delay)

        # ⑦ Early exit if all PoIs surveyed and queue empty
        from core.models import PoIStatus
        all_done = all(p.status == PoIStatus.SURVEYED for p in pois)
        if all_done and not manager.data_queue:
            print(f"\n  ✅ Mission complete at tick {tick}!")
            break

    # ── Final Frame ───────────────────────────────────────
    if not args.no_viz:
        os.system("clear" if os.name == "posix" else "cls")
        frame = render_frame(args.ticks, uavs, pois, gcs, network)
        print(frame)

    # ── Mission Summary ───────────────────────────────────
    summary = manager.mission_summary()
    print("\n" + "═" * 60)
    print("  MISSION SUMMARY")
    print("═" * 60)
    for k, v in summary.items():
        print(f"  {k:<30} {v}")

    # ── Acceptance Report ─────────────────────────────────
    if not args.no_report:
        print("\n" + "═" * 60)
        print("  RUNNING ACCEPTANCE TESTS (EchoRescue-style)")
        print("═" * 60)
        all_pass = reporter.run_all_checks()
        for result in reporter.results:
            icon = "✅" if result["passed"] else "❌"
            print(f"  {icon} {result['test']:<35} {result['evidence']}")

        print("\n" + ("  🏆 ALL TESTS PASSED" if all_pass else "  ⚠️  SOME TESTS FAILED"))

        # Save reports
        report_dir = os.path.join(os.path.dirname(__file__), "..", "logs")
        reporter.save_report(output_dir=report_dir)

    print("\n  Simulation complete.\n")


if __name__ == "__main__":
    main()
