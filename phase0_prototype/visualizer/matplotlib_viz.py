"""
UAV-X Phase 0: Matplotlib Mission Visualizer
=============================================
Generates a 2×2 dark-themed summary plot from a completed simulation run.

Subplots
--------
1. 2D Mission Map      — UAV final positions, PoI markers, GCS star, path
                         traces, relay chain, legend.
2. Battery Over Time   — Per-UAV battery drain curves with RTL threshold line.
3. Network Connectivity — Area chart of GCS-reachable node fraction over time.
4. Data Delivery Timeline — Delivery tick vs. priority scatter.

Usage
-----
Run standalone to execute a 200-tick headless simulation and save the plot::

    python visualizer/matplotlib_viz.py

Or import and call from another module::

    from visualizer.matplotlib_viz import MissionVisualizer, run_and_visualize
    run_and_visualize()
"""

from __future__ import annotations

import os
import sys
import random
from typing import Dict, List, Any

# Make sure phase0_prototype modules are importable when run directly
_HERE = os.path.dirname(__file__)
_PHASE0 = os.path.abspath(os.path.join(_HERE, ".."))
if _PHASE0 not in sys.path:
    sys.path.insert(0, _PHASE0)

import matplotlib
matplotlib.use("Agg")   # non-interactive backend — safe in any environment
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from matplotlib.collections import LineCollection
import numpy as np


# ── Colour constants (mirror dashboard palette) ────────────────────────────────
_BG        = "#0a0e1a"
_PANEL     = "#0d1224"
_GRID      = "#1a2040"
_ACCENT    = "#00d4ff"
_GREEN     = "#00ff88"
_RED       = "#ff4444"
_ORANGE    = "#ff8c00"
_YELLOW    = "#ffd700"
_BLUE      = "#4488ff"
_GRAY      = "#8899aa"
_WHITE     = "#c8d8f0"

_ROLE_COLORS = {
    "Scout": _ACCENT,
    "Relay": _BLUE,
    "Idle":  _GRAY,
    "RTL":   _ORANGE,
}

_PRIORITY_COLORS = {
    "CRITICAL": _RED,
    "HIGH":     _ORANGE,
    "MEDIUM":   _YELLOW,
    "LOW":      _GREEN,
}

_PRIORITY_INT = {
    "CRITICAL": 0,
    "HIGH":     1,
    "MEDIUM":   2,
    "LOW":      3,
}


# ──────────────────────────────────────────────────────────────────────────────
class MissionVisualizer:
    """
    Post-mission visualizer that produces a 2×2 matplotlib figure.

    Parameters
    ----------
    manager : MissionManager
        The finished MissionManager instance (carries UAV, PoI, GCS state).
    network : CommNetwork
        The finished CommNetwork instance.
    history : list of dict
        Tick-by-tick snapshots with the structure::

            {
                "tick": int,
                "uav_states": {
                    "<uav_id>": {"x": float, "y": float,
                                 "battery": float, "role": str}
                },
                "reachable_count": int,   # UAVs with path to GCS
                "packets_delivered": int, # cumulative GCS-received packets
                "delivery_events": [      # packets delivered this tick
                    {"tick": int, "priority": str}
                ]
            }
    """

    def __init__(self, manager, network, history: List[Dict[str, Any]]):
        self.manager = manager
        self.network = network
        self.history = history

    # ── Public API ────────────────────────────────────────────────────────────

    def generate_all_plots(self, output_path: str = "mission_summary.png"):
        """
        Generate and save the 2×2 mission summary figure.

        Parameters
        ----------
        output_path : str
            File path (PNG/PDF/SVG) where the figure is saved.
        """
        fig = plt.figure(figsize=(14, 10), facecolor=_BG)
        fig.suptitle(
            "UAV-X PHASE 0 — MISSION SUMMARY",
            color=_ACCENT, fontsize=14, fontweight="bold",
            fontfamily="monospace", y=0.97,
        )

        axes = [
            fig.add_subplot(2, 2, 1),
            fig.add_subplot(2, 2, 2),
            fig.add_subplot(2, 2, 3),
            fig.add_subplot(2, 2, 4),
        ]

        self._style_ax(axes[0], "2D Mission Map")
        self._style_ax(axes[1], "Battery Levels Over Time")
        self._style_ax(axes[2], "Network Connectivity")
        self._style_ax(axes[3], "Data Delivery Timeline")

        self._plot_mission_map(axes[0])
        self._plot_battery(axes[1])
        self._plot_connectivity(axes[2])
        self._plot_delivery_timeline(axes[3])

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.savefig(output_path, dpi=120, facecolor=_BG, bbox_inches="tight")
        plt.close(fig)
        print(f"[MissionVisualizer] Plot saved → {output_path}")

    # ── Styling helper ────────────────────────────────────────────────────────

    def _style_ax(self, ax: plt.Axes, title: str):
        """Apply dark theme styling to an axes object."""
        ax.set_facecolor(_PANEL)
        ax.set_title(title, color=_ACCENT, fontsize=10,
                     fontfamily="monospace", pad=8)
        ax.tick_params(colors=_GRAY, labelsize=7)
        for spine in ax.spines.values():
            spine.set_edgecolor(_GRID)
        ax.grid(True, color=_GRID, linewidth=0.5, linestyle="--", alpha=0.6)
        ax.xaxis.label.set_color(_GRAY)
        ax.yaxis.label.set_color(_GRAY)

    # ── Subplot 1: 2D Mission Map ─────────────────────────────────────────────

    def _plot_mission_map(self, ax: plt.Axes):
        """
        Scatter-plot the final UAV positions, PoI markers, GCS star, path
        traces from history snapshots, and relay chain as dashed lines.
        """
        uavs = self.manager.uavs
        pois = self.manager.pois
        gcs  = self.manager.gcs

        # ── Path traces from history (thin lines per UAV) ────────────────
        uav_ids = list(uavs.keys())
        colors_by_id: Dict[str, str] = {}
        for uid in uav_ids:
            last_role = uavs[uid].role.value
            colors_by_id[uid] = _ROLE_COLORS.get(last_role, _GRAY)

        path_xs: Dict[str, List[float]] = {uid: [] for uid in uav_ids}
        path_ys: Dict[str, List[float]] = {uid: [] for uid in uav_ids}
        for snap in self.history:
            for uid in uav_ids:
                state = snap["uav_states"].get(uid)
                if state:
                    path_xs[uid].append(state["x"])
                    path_ys[uid].append(state["y"])

        for uid in uav_ids:
            ax.plot(
                path_xs[uid], path_ys[uid],
                color=colors_by_id[uid], linewidth=0.5, alpha=0.35, zorder=1,
            )

        # ── Relay chain (dashed) ──────────────────────────────────────────
        from core.models import UAVRole
        relay_uavs = [u for u in uavs.values() if u.role == UAVRole.RELAY]
        relay_uavs_sorted = sorted(
            relay_uavs,
            key=lambda u: u.position.x + u.position.y,
        )
        chain_pts = [(gcs.position.x, gcs.position.y)] + [
            (u.position.x, u.position.y) for u in relay_uavs_sorted
        ]
        if len(chain_pts) > 1:
            cxs = [p[0] for p in chain_pts]
            cys = [p[1] for p in chain_pts]
            ax.plot(cxs, cys, color=_ACCENT, linewidth=1,
                    linestyle="--", alpha=0.5, zorder=2)

        # ── PoI markers ──────────────────────────────────────────────────
        for poi in pois:
            col  = _PRIORITY_COLORS.get(poi.priority.name, _GRAY)
            marker = "o"
            edgecol = col
            size = 120
            facecolor = col if poi.status.name == "SURVEYED" else "none"
            ax.scatter(
                poi.position.x, poi.position.y,
                s=size, c=facecolor if facecolor != "none" else "none",
                edgecolors=edgecol, linewidths=1.8,
                marker=marker, zorder=4, alpha=0.9,
            )
            ax.annotate(
                poi.poi_id, (poi.position.x, poi.position.y),
                xytext=(0, 10), textcoords="offset points",
                fontsize=6, color=col, fontfamily="monospace",
                ha="center",
            )

        # ── UAV final positions ──────────────────────────────────────────
        for uid, uav in uavs.items():
            col = _ROLE_COLORS.get(uav.role.value, _GRAY)
            if uav.status.value == "Failed":
                col = _RED
                marker = "X"
            else:
                marker = "^"
            ax.scatter(
                uav.position.x, uav.position.y,
                s=80, c=col, marker=marker, zorder=5, alpha=0.95,
                edgecolors="white", linewidths=0.4,
            )

        # ── GCS star ─────────────────────────────────────────────────────
        ax.scatter(
            gcs.position.x, gcs.position.y,
            s=200, c=_ACCENT, marker="*", zorder=6,
            edgecolors="white", linewidths=0.4,
            label="GCS",
        )
        ax.annotate(
            "GCS", (gcs.position.x, gcs.position.y),
            xytext=(12, -4), textcoords="offset points",
            fontsize=7, color=_ACCENT, fontfamily="monospace",
        )

        # ── Legend ────────────────────────────────────────────────────────
        legend_items = [
            mlines.Line2D([], [], color=_ACCENT, marker="^", linestyle="None",
                          markersize=7, label="Scout"),
            mlines.Line2D([], [], color=_BLUE,   marker="^", linestyle="None",
                          markersize=7, label="Relay"),
            mlines.Line2D([], [], color=_ORANGE, marker="^", linestyle="None",
                          markersize=7, label="RTL"),
            mlines.Line2D([], [], color=_RED,    marker="X", linestyle="None",
                          markersize=7, label="Failed"),
            mpatches.Patch(facecolor="none", edgecolor=_RED,    label="CRITICAL PoI"),
            mpatches.Patch(facecolor="none", edgecolor=_ORANGE, label="HIGH PoI"),
            mpatches.Patch(facecolor="none", edgecolor=_YELLOW, label="MEDIUM PoI"),
            mpatches.Patch(facecolor="none", edgecolor=_GREEN,  label="LOW PoI"),
        ]
        ax.legend(
            handles=legend_items, loc="upper right",
            fontsize=6, framealpha=0.3, facecolor=_BG,
            edgecolor=_GRID, labelcolor=_GRAY,
        )

        ax.set_xlabel("x (metres east)", fontsize=7)
        ax.set_ylabel("y (metres north)", fontsize=7)
        ax.set_xlim(-80, 2280)
        ax.set_ylim(-80, 1480)

    # ── Subplot 2: Battery Over Time ──────────────────────────────────────────

    def _plot_battery(self, ax: plt.Axes):
        """
        Plot one battery-level curve per UAV over all history ticks.
        A horizontal dashed red line marks the RTL threshold (25 %).
        """
        if not self.history:
            ax.text(0.5, 0.5, "No history data", transform=ax.transAxes,
                    color=_GRAY, ha="center", va="center")
            return

        uav_ids = list(self.manager.uavs.keys())
        ticks   = [snap["tick"] for snap in self.history]

        # Generate distinct hues for UAVs
        cmap = plt.cm.get_cmap("cool", len(uav_ids))
        for i, uid in enumerate(uav_ids):
            bats = []
            for snap in self.history:
                state = snap["uav_states"].get(uid)
                bats.append(state["battery"] if state else float("nan"))
            ax.plot(
                ticks, bats,
                label=uid, linewidth=1.2,
                color=cmap(i), alpha=0.85,
            )

        # RTL threshold line
        ax.axhline(
            y=25, color=_RED, linewidth=1.0,
            linestyle="--", alpha=0.7, label="RTL threshold",
        )

        ax.set_xlabel("Simulation Tick", fontsize=7)
        ax.set_ylabel("Battery (%)", fontsize=7)
        ax.set_ylim(-5, 105)
        ax.legend(
            loc="upper right", fontsize=5, ncol=2,
            framealpha=0.3, facecolor=_BG,
            edgecolor=_GRID, labelcolor=_GRAY,
        )

    # ── Subplot 3: Network Connectivity ───────────────────────────────────────

    def _plot_connectivity(self, ax: plt.Axes):
        """
        Area chart showing the fraction of UAVs reachable to GCS each tick.
        """
        if not self.history:
            ax.text(0.5, 0.5, "No history data", transform=ax.transAxes,
                    color=_GRAY, ha="center", va="center")
            return

        total = max(len(self.manager.uavs), 1)
        ticks  = [snap["tick"] for snap in self.history]
        fracs  = [snap.get("reachable_count", 0) / total for snap in self.history]

        ax.fill_between(ticks, fracs, alpha=0.3, color=_ACCENT)
        ax.plot(ticks, fracs, color=_ACCENT, linewidth=1.4)

        ax.set_xlabel("Simulation Tick", fontsize=7)
        ax.set_ylabel("Fraction Reachable to GCS", fontsize=7)
        ax.set_ylim(-0.05, 1.1)
        ax.yaxis.set_major_formatter(
            matplotlib.ticker.FuncFormatter(lambda v, _: f"{int(v*100)}%")
        )

    # ── Subplot 4: Data Delivery Timeline ─────────────────────────────────────

    def _plot_delivery_timeline(self, ax: plt.Axes):
        """
        Horizontal scatter: x = delivery tick, y = priority level (0–3),
        coloured by priority, marker size = 50.
        """
        delivery_ticks: List[float]  = []
        priorities_num: List[int]    = []
        colors_list:    List[str]    = []
        labels_seen:    set          = set()

        for snap in self.history:
            for ev in snap.get("delivery_events", []):
                p_name = ev.get("priority", "LOW")
                delivery_ticks.append(ev.get("tick", snap["tick"]))
                priorities_num.append(_PRIORITY_INT.get(p_name, 3))
                colors_list.append(_PRIORITY_COLORS.get(p_name, _GRAY))
                labels_seen.add(p_name)

        if not delivery_ticks:
            ax.text(0.5, 0.5, "No packets delivered yet",
                    transform=ax.transAxes, color=_GRAY,
                    ha="center", va="center")
            return

        ax.scatter(
            delivery_ticks, priorities_num,
            c=colors_list, s=50, alpha=0.85, zorder=3,
            edgecolors="none",
        )

        ax.set_xlabel("Delivery Tick", fontsize=7)
        ax.set_ylabel("Priority Level", fontsize=7)
        ax.set_yticks([0, 1, 2, 3])
        ax.set_yticklabels(["CRITICAL", "HIGH", "MEDIUM", "LOW"],
                           fontsize=6, color=_GRAY)

        # Compact legend for priority colours
        legend_items = [
            mpatches.Patch(color=_PRIORITY_COLORS[p], label=p)
            for p in ["CRITICAL", "HIGH", "MEDIUM", "LOW"]
            if p in labels_seen
        ]
        if legend_items:
            ax.legend(
                handles=legend_items, loc="upper right",
                fontsize=6, framealpha=0.3, facecolor=_BG,
                edgecolor=_GRID, labelcolor=_GRAY,
            )


# ──────────────────────────────────────────────────────────────────────────────
#  Standalone runner
# ──────────────────────────────────────────────────────────────────────────────

def run_and_visualize(output_path: str = "mission_summary.png", ticks: int = 200):
    """
    Import the scenario modules, run a headless 200-tick simulation while
    collecting per-tick history snapshots, then call
    :meth:`MissionVisualizer.generate_all_plots`.

    Parameters
    ----------
    output_path : str
        Destination PNG path for the saved figure.
    ticks : int
        Number of simulation ticks to run.
    """
    from configs.scenario import create_disaster_scenario
    from core.comm_network import CommNetwork
    from core.mission_manager import MissionManager
    from core.models import UAVStatus, UAVRole, PoIStatus

    print("[run_and_visualize] Initialising scenario…")
    gcs, uavs, pois = create_disaster_scenario()

    network = CommNetwork(uavs=uavs, gcs=gcs, max_range=800.0, noise_enabled=True)
    manager = MissionManager(
        uavs=uavs, pois=pois, gcs=gcs, network=network,
        relay_altitude=50.0, relay_spacing=400.0,
    )

    history: List[Dict[str, Any]] = []

    print(f"[run_and_visualize] Running {ticks} ticks…")
    for tick in range(ticks):

        # Optional fault injection at tick 80
        if tick == 80:
            target = uavs.get("UAV-05")
            if target:
                target.status = UAVStatus.FAILED
                target.role   = UAVRole.IDLE
                manager.event_log.append({
                    "tick": tick, "type": "FAULT_INJECTED",
                    "actor": "UAV-05",
                    "message": "Relay UAV-05 intentionally killed — testing recovery",
                })

        manager.tick(tick)
        manager.move_uavs(tick, dt=1.0)
        network.update_links()
        network.compute_routing_table()

        # Collect delivery events from this tick
        delivery_events = [
            {
                "tick": e["tick"],
                "priority": next(
                    (pk.priority.name for pk in gcs.received_packets
                     if pk.source_uav == e["actor"]),
                    "LOW",
                ),
            }
            for e in manager.event_log
            if e.get("type") == "DELIVERED" and e.get("tick") == tick
        ]

        manager.flush_data_queue(tick)

        conn = network.get_connectivity_report()
        snap: Dict[str, Any] = {
            "tick": tick,
            "uav_states": {
                uid: {
                    "x":       uav.position.x,
                    "y":       uav.position.y,
                    "battery": uav.battery,
                    "role":    uav.role.value,
                }
                for uid, uav in uavs.items()
            },
            "reachable_count": conn.get("reachable_to_gcs", 0),
            "packets_delivered": len(gcs.received_packets),
            "delivery_events": delivery_events,
        }
        history.append(snap)

        # Early exit
        if all(p.status == PoIStatus.SURVEYED for p in pois) and not manager.data_queue:
            print(f"[run_and_visualize] Mission complete at tick {tick}.")
            break

    print(f"[run_and_visualize] Simulation done. Generating plots…")
    viz = MissionVisualizer(manager=manager, network=network, history=history)
    viz.generate_all_plots(output_path=output_path)


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    run_and_visualize()
