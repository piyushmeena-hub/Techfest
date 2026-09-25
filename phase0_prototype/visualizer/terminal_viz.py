"""
UAV-X Phase 0: Terminal Visualizer
Renders real-time ASCII simulation of the disaster zone, UAV positions,
relay chain, and comms links — no heavy graphics libraries needed.
"""
from __future__ import annotations
import os
import math
from typing import Dict, List
from core.models import UAV, UAVRole, UAVStatus, PointOfInterest, PoIStatus, GroundControlStation
from core.comm_network import CommNetwork, LinkStatus


# Grid dimensions
GRID_W = 60
GRID_H = 25
WORLD_W = 2200.0   # metres
WORLD_H = 1400.0   # metres

ROLE_ICONS = {
    UAVRole.SCOUT: "✈",
    UAVRole.RELAY: "📡",
    UAVRole.IDLE:  "⬜",
    UAVRole.RTL:   "🔙",
}

POI_ICONS = {
    PoIStatus.PENDING:  "○",
    PoIStatus.ASSIGNED: "◉",
    PoIStatus.SURVEYED: "●",
}


def world_to_grid(x: float, y: float) -> tuple:
    gx = int((x / WORLD_W) * (GRID_W - 1))
    gy = int((1.0 - y / WORLD_H) * (GRID_H - 1))
    return max(0, min(GRID_W - 1, gx)), max(0, min(GRID_H - 1, gy))


def render_frame(tick: int, uavs: Dict[str, UAV],
                 pois: List[PointOfInterest],
                 gcs: GroundControlStation,
                 network: CommNetwork) -> str:
    # Build empty grid
    grid = [["·"] * GRID_W for _ in range(GRID_H)]

    # Draw border
    for x in range(GRID_W):
        grid[0][x] = "─"
        grid[GRID_H - 1][x] = "─"
    for y in range(GRID_H):
        grid[y][0] = "│"
        grid[y][GRID_W - 1] = "│"
    grid[0][0] = "┌"; grid[0][GRID_W-1] = "┐"
    grid[GRID_H-1][0] = "└"; grid[GRID_H-1][GRID_W-1] = "┘"

    # Draw GCS
    gx, gy = world_to_grid(gcs.position.x, gcs.position.y)
    grid[gy][gx] = "G"

    # Draw PoIs
    for poi in pois:
        px, py = world_to_grid(poi.position.x, poi.position.y)
        icon = POI_ICONS.get(poi.status, "?")
        # Use ASCII fallback
        ascii_icon = {"○": "o", "◉": "O", "●": "*"}.get(icon, "?")
        grid[py][px] = ascii_icon

    # Draw UAVs
    for uav in uavs.values():
        ux, uy = world_to_grid(uav.position.x, uav.position.y)
        if uav.status == UAVStatus.FAILED:
            grid[uy][ux] = "X"
        elif uav.role == UAVRole.RELAY:
            grid[uy][ux] = "R"
        elif uav.role == UAVRole.SCOUT:
            grid[uy][ux] = "S"
        elif uav.role == UAVRole.RTL:
            grid[uy][ux] = "<"
        else:
            grid[uy][ux] = "I"

    # Build grid string
    grid_str = "\n".join("".join(row) for row in grid)

    # Build stats panel
    active = sum(1 for u in uavs.values() if u.status == UAVStatus.ACTIVE)
    scouts = sum(1 for u in uavs.values() if u.role == UAVRole.SCOUT)
    relays = sum(1 for u in uavs.values() if u.role == UAVRole.RELAY)
    rtl_c  = sum(1 for u in uavs.values() if u.role == UAVRole.RTL)
    failed = sum(1 for u in uavs.values() if u.status == UAVStatus.FAILED)
    surveyed = sum(1 for p in pois if p.status == PoIStatus.SURVEYED)
    delivered = len(gcs.received_packets)
    net = network.get_connectivity_report()

    bat_bars = ""
    for uid, uav in sorted(uavs.items()):
        bar_len = int(uav.battery / 10)
        bar = "█" * bar_len + "░" * (10 - bar_len)
        role_char = uav.role.name[0]
        status_char = "✗" if uav.status == UAVStatus.FAILED else " "
        bat_bars += f"  {uid}: [{bar}] {uav.battery:5.1f}% {role_char}{status_char}\n"

    header = f"╔══ UAV-X DISASTER RESPONSE SIMULATION — TICK {tick:04d} ══╗"
    legend = "  G=GCS  S=Scout  R=Relay  <=RTL  X=Failed  o=PoI pending  O=assigned  *=surveyed"

    output = f"""
{header}
{grid_str}
{legend}

  Fleet Status ({active}/{len(uavs)} active | {scouts} scouting | {relays} relaying | {rtl_c} RTL | {failed} failed):
{bat_bars}
  Mission: {surveyed}/{len(pois)} PoIs surveyed | {delivered} packets at GCS
  Network: {net['reachable_to_gcs']}/{net['total_nodes']} UAVs reachable | {net['active_links']} links up | {net['weak_links']} weak
  Isolated: {net['isolated_nodes'] or 'None'}
"""
    return output
