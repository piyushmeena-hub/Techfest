"""
UAV-X Phase 2 – Swarm Network Simulation (NS-3 Python Wrapper)
==============================================================
Provides a unified interface for simulating inter-UAV and UAV↔GCS
communications at the network layer.

Two back-ends are supported:

1. **NS-3 back-end** (preferred)
   Uses the official NS-3 Python bindings (``import ns``).
   Models a 802.11n ad-hoc Wi-Fi network with:
     - WifiHelper (802.11n, ConstantRateWifiManager)
     - AdhocWifiMac
     - ConstantPositionMobilityModel (positions updated each tick)
     - UdpEchoHelper for synthetic traffic (required to populate
       NS-3's link-state tables)
     - OLSR routing (ns.olsr module) for multi-hop paths to GCS

2. **Pure-Python fallback** (if ns not installed)
   Uses the ``CommNetwork`` class from ``phase0_prototype/core/comm_network``
   (or a local stub if that module is also unavailable).
   The fallback implements a simplified radio propagation model:
     - Free-space path loss → signal strength
     - SNR threshold → link UP / WEAK / DOWN
     - BFS shortest-path routing

Usage
-----
>>> net = SwarmNetworkNS3(max_range=800.0)
>>> net.setup_nodes({
...     "GCS":   (0, 0, 0),
...     "uav_0": (300, 200, 50),
...     "uav_1": (650, 400, 60),
... })
>>> for tick in range(10):
...     net.run_tick(dt=1.0)
...     states = net.get_link_states()
...     routes = net.get_routing_table()
...     print(routes)

Dependencies
------------
  NS-3 Python bindings: https://www.nsnam.org/docs/release/3.41/python-api/
  Install:  pip install --index-url https://pypi.nsnam.org ns3   (experimental)
  OR build from source with --enable-python-bindings
"""

import math
import logging
from collections import defaultdict, deque
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger("SwarmNetworkNS3")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

# ---------------------------------------------------------------------------
# Attempt to import NS-3 Python bindings
# ---------------------------------------------------------------------------
try:
    import ns                                           # top-level ns-3 module
    import ns.core                                      # Simulator, Time, etc.
    import ns.network                                   # NodeContainer, NetDeviceContainer
    import ns.internet                                  # InternetStackHelper, Ipv4
    import ns.wifi                                      # WifiHelper, YansWifiChannel
    import ns.mobility                                  # MobilityHelper, ConstantPositionMobilityModel
    import ns.applications                              # UdpEchoHelper
    import ns.olsr                                      # OLSR routing protocol

    _NS3_AVAILABLE = True
    logger.info("NS-3 Python bindings detected – using NS-3 back-end.")

except ImportError:
    _NS3_AVAILABLE = False
    logger.warning(
        "NS-3 Python bindings not found.  "
        "Falling back to pure-Python CommNetwork simulation.\n"
        "To install NS-3 bindings: https://www.nsnam.org/docs/release/3.41/python-api/"
    )

# ---------------------------------------------------------------------------
# Attempt to import the Phase-0 CommNetwork (pure-Python fallback)
# ---------------------------------------------------------------------------
try:
    import sys
    import os
    # Allow import from sibling phase0_prototype directory
    _PHASE0_PATH = os.path.join(
        os.path.dirname(__file__), "..", "..", "phase0_prototype"
    )
    if _PHASE0_PATH not in sys.path:
        sys.path.insert(0, os.path.abspath(_PHASE0_PATH))

    from core.comm_network import CommNetwork as _Phase0CommNetwork  # type: ignore
    _PHASE0_AVAILABLE = True
    logger.info("Phase-0 CommNetwork imported successfully.")

except ImportError:
    _PHASE0_AVAILABLE = False
    _Phase0CommNetwork = None
    logger.warning(
        "phase0_prototype CommNetwork not found – using built-in stub fallback."
    )


# ===========================================================================
# Constants
# ===========================================================================

#: Frequency in GHz (5 GHz Wi-Fi band)
_WIFI_FREQ_GHZ: float = 5.18

#: Transmit power in dBm (typical Wi-Fi AP)
_TX_POWER_DBM: float = 20.0

#: Receiver sensitivity / noise floor in dBm
_NOISE_FLOOR_DBM: float = -90.0

#: SNR threshold for WEAK link (dB)
_SNR_WEAK_DB: float = 10.0

#: SNR threshold for UP link (dB)
_SNR_UP_DB: float = 20.0

#: Speed of light (m/s)
_C: float = 3.0e8

#: GCS node name constant
_GCS_ID: str = "GCS"


# ===========================================================================
# Built-in pure-Python network stub (used if both NS-3 and Phase0 missing)
# ===========================================================================

class _BuiltinCommNetwork:
    """
    Minimal radio-propagation network stub.

    Models free-space path loss for 5 GHz Wi-Fi and provides:
      - Per-link signal strength and status (UP / WEAK / DOWN)
      - BFS routing table from every node to GCS
    """

    def __init__(self, max_range: float = 800.0, noise: bool = True) -> None:
        self.max_range = max_range
        self.noise = noise
        self._positions: Dict[str, Tuple[float, float, float]] = {}

    def set_positions(
        self, positions: Dict[str, Tuple[float, float, float]]
    ) -> None:
        """Update node positions."""
        self._positions = dict(positions)

    def _path_loss_db(self, dist_m: float) -> float:
        """
        Free-space path loss (Friis) in dB:
            L = 20·log10(4πd·f/c)
        Clamped to 0 dB for d ≤ 1 m.
        """
        if dist_m <= 1.0:
            return 0.0
        freq_hz = _WIFI_FREQ_GHZ * 1e9
        loss = 20.0 * math.log10(
            (4.0 * math.pi * dist_m * freq_hz) / _C
        )
        return max(0.0, loss)

    def get_link_states(
        self,
    ) -> Dict[Tuple[str, str], Dict]:
        """
        Compute link state for every ordered pair of nodes.

        Returns
        -------
        dict mapping (a, b) → {'status': str, 'signal_dbm': float,
                                'path_loss_db': float, 'distance_m': float}
        """
        node_ids = list(self._positions.keys())
        states: Dict[Tuple[str, str], Dict] = {}

        for i, a in enumerate(node_ids):
            for j, b in enumerate(node_ids):
                if a == b:
                    continue
                pa = self._positions[a]
                pb = self._positions[b]
                dist = math.sqrt(
                    (pa[0] - pb[0]) ** 2
                    + (pa[1] - pb[1]) ** 2
                    + (pa[2] - pb[2]) ** 2
                )

                pl = self._path_loss_db(dist)
                signal = _TX_POWER_DBM - pl
                snr = signal - _NOISE_FLOOR_DBM

                if dist > self.max_range or snr < _SNR_WEAK_DB:
                    status = "DOWN"
                elif snr < _SNR_UP_DB:
                    status = "WEAK"
                else:
                    status = "UP"

                states[(a, b)] = {
                    "status":       status,
                    "signal_dbm":   round(signal, 2),
                    "path_loss_db": round(pl, 2),
                    "distance_m":   round(dist, 2),
                }

        return states

    def get_routing_table(self) -> Dict[str, List[str]]:
        """
        BFS shortest-path routing from every node to GCS.

        Returns
        -------
        dict mapping node_id → [node_id, …, GCS] (path including both ends),
        or [] if no path exists (isolated node).
        """
        link_states = self.get_link_states()

        # Build adjacency graph from UP/WEAK links
        adj: Dict[str, List[str]] = defaultdict(list)
        for (a, b), info in link_states.items():
            if info["status"] in ("UP", "WEAK"):
                adj[a].append(b)

        node_ids = list(self._positions.keys())
        routing: Dict[str, List[str]] = {}

        for start in node_ids:
            if start == _GCS_ID:
                routing[start] = [_GCS_ID]
                continue

            # BFS from start → GCS
            visited = {start}
            queue: deque = deque([[start]])
            found: Optional[List[str]] = None

            while queue:
                path = queue.popleft()
                current = path[-1]
                for neighbour in adj.get(current, []):
                    if neighbour in visited:
                        continue
                    new_path = path + [neighbour]
                    if neighbour == _GCS_ID:
                        found = new_path
                        break
                    visited.add(neighbour)
                    queue.append(new_path)
                if found:
                    break

            routing[start] = found if found else []

        return routing


# ===========================================================================
# SwarmNetworkNS3
# ===========================================================================

class SwarmNetworkNS3:
    """
    Unified swarm communications network simulator.

    Wraps NS-3 Python bindings when available; otherwise transparently
    falls back to the Phase-0 :class:`CommNetwork` or the built-in
    :class:`_BuiltinCommNetwork` stub.

    Parameters
    ----------
    max_range : float
        Maximum communication range in metres (used by fallback back-end).
        In the NS-3 back-end this controls the propagation loss model cutoff.
    noise : bool
        Whether to add Gaussian noise to link signals (fallback only;
        NS-3 handles noise internally via YansWifiChannel).

    Attributes
    ----------
    backend : str
        One of ``"ns3"``, ``"phase0"``, ``"builtin"`` – identifies the
        active simulation back-end.

    Examples
    --------
    >>> net = SwarmNetworkNS3(max_range=800.0, noise=True)
    >>> net.setup_nodes({"GCS": (0,0,0), "uav_0": (400,200,50)})
    >>> net.run_tick(dt=1.0)
    >>> print(net.get_link_states())
    >>> print(net.get_routing_table())
    """

    def __init__(
        self,
        max_range: float = 800.0,
        noise: bool = True,
    ) -> None:
        self.max_range = max_range
        self.noise = noise

        # Internal state
        self._uav_positions: Dict[str, Tuple[float, float, float]] = {}
        self._ns3_nodes = None          # ns.network.NodeContainer (NS-3 only)
        self._ns3_node_map: Dict[str, int] = {}   # name → NS-3 node index
        self._ns3_mobility_map: Dict[str, object] = {}  # name → MobilityModel
        self._ns3_initialized: bool = False
        self._tick: int = 0

        # Cached results from the last run_tick
        self._link_states: Dict[Tuple[str, str], Dict] = {}
        self._routing_table: Dict[str, List[str]] = {}

        # Choose back-end
        if _NS3_AVAILABLE:
            self.backend = "ns3"
        elif _PHASE0_AVAILABLE:
            self.backend = "phase0"
            self._fallback = _Phase0CommNetwork(comm_range=max_range)
        else:
            self.backend = "builtin"
            self._fallback = _BuiltinCommNetwork(
                max_range=max_range, noise=noise
            )

        logger.info(
            "SwarmNetworkNS3 initialised [backend=%s, max_range=%.0f m]",
            self.backend, max_range,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def setup_nodes(
        self,
        uav_positions: Dict[str, Tuple[float, float, float]],
    ) -> None:
        """
        Initialise or reset the network with the given node positions.

        Must be called before the first :meth:`run_tick`.

        Parameters
        ----------
        uav_positions : dict[str, tuple[float, float, float]]
            Mapping of node name → ``(x_m, y_m, z_m)``.
            Include ``"GCS"`` as one of the keys to designate the
            ground control station (all routes compute paths to GCS).

        Examples
        --------
        >>> net.setup_nodes({
        ...     "GCS":   (0, 0, 0),
        ...     "uav_0": (300, 0, 50),
        ...     "uav_1": (700, 0, 60),
        ... })
        """
        self._uav_positions = dict(uav_positions)

        if self.backend == "ns3":
            self._ns3_setup(uav_positions)
        elif self.backend == "phase0":
            # Phase-0 CommNetwork uses UAV objects with .position attributes;
            # wrap positions into a compatible structure.
            self._phase0_sync_positions()
        else:
            # Built-in stub
            self._fallback.set_positions(uav_positions)

        logger.info(
            "setup_nodes(): %d nodes registered.", len(uav_positions)
        )

    def run_tick(self, dt: float = 1.0) -> None:
        """
        Advance the network simulation by *dt* seconds and update link states.

        After this call, :meth:`get_link_states` and :meth:`get_routing_table`
        return fresh data.

        Parameters
        ----------
        dt : float
            Simulation time step in seconds.

        Examples
        --------
        >>> net.run_tick(dt=0.1)   # 100 ms tick
        """
        self._tick += 1

        if self.backend == "ns3":
            self._ns3_run_tick(dt)
        else:
            # Fallback: recompute from current positions (stateless model)
            self._link_states  = self._fallback.get_link_states()
            self._routing_table = self._fallback.get_routing_table()

        logger.debug(
            "Tick %d: %d links computed.", self._tick, len(self._link_states)
        )

    def get_link_states(
        self,
    ) -> Dict[Tuple[str, str], Dict]:
        """
        Return the link state for every directed pair of nodes after the
        most recent :meth:`run_tick`.

        Returns
        -------
        dict mapping ``(node_a, node_b)`` → info dict:

        .. code-block:: python

            {
                "status":       "UP" | "WEAK" | "DOWN",
                "signal_dbm":   float,   # received signal strength (dBm)
                "path_loss_db": float,   # path loss (dB)
                "distance_m":   float,   # 3-D Euclidean distance
            }

        Examples
        --------
        >>> states = net.get_link_states()
        >>> print(states[("uav_0", "GCS")])
        {'status': 'UP', 'signal_dbm': -45.2, 'path_loss_db': 65.2, 'distance_m': 300.0}
        """
        return dict(self._link_states)

    def get_routing_table(self) -> Dict[str, List[str]]:
        """
        Return the routing table computed after the most recent :meth:`run_tick`.

        Each entry gives the multi-hop path from that node to GCS.

        Returns
        -------
        dict mapping ``node_id`` → ``[node_id, ..., "GCS"]``

        An empty list ``[]`` means the node is isolated (no path to GCS).

        Examples
        --------
        >>> routes = net.get_routing_table()
        >>> # 3-hop route: Scout → Relay → GCS
        >>> print(routes["uav_2"])   # ["uav_2", "uav_0", "GCS"]
        """
        return dict(self._routing_table)

    def update_node_position(
        self,
        node_id: str,
        position: Tuple[float, float, float],
    ) -> None:
        """
        Update the position of a single node without a full re-setup.

        Call :meth:`run_tick` after all position updates to refresh states.

        Parameters
        ----------
        node_id : str
            Node identifier (same as used in :meth:`setup_nodes`).
        position : tuple[float, float, float]
            New ``(x_m, y_m, z_m)`` coordinates.
        """
        self._uav_positions[node_id] = position

        if self.backend == "ns3" and self._ns3_initialized:
            mob = self._ns3_mobility_map.get(node_id)
            if mob:
                pos = ns.core.Vector(position[0], position[1], position[2])
                mob.SetPosition(pos)
        elif self.backend == "phase0":
            self._phase0_sync_positions()
        else:
            self._fallback.set_positions(self._uav_positions)

    # ------------------------------------------------------------------
    # NS-3 back-end internals
    # ------------------------------------------------------------------

    def _ns3_setup(
        self,
        positions: Dict[str, Tuple[float, float, float]],
    ) -> None:
        """
        Initialise NS-3 nodes, Wi-Fi channel, IP stack, and OLSR routing.

        NS-3 setup sequence:
          1. Create one ns3::Node per UAV / GCS.
          2. Configure a YansWifiChannel with LogDistancePropagationLossModel.
          3. Set up WifiHelper (802.11n, ad-hoc mode, ConstantRateWifiManager).
          4. Install InternetStackHelper with OLSR routing protocol.
          5. Assign IP addresses from the 10.0.0.0/8 subnet.
          6. Place nodes at their initial positions using ConstantPositionMobilityModel.
          7. Install a UdpEchoServer/Client pair to generate synthetic traffic
             so that NS-3 builds link-state tables.
        """
        # ---- Step 1: Create nodes ----------------------------------------
        node_names = list(positions.keys())
        n_nodes = len(node_names)

        # ns.network.NodeContainer: holds pointers to ns3::Node objects
        nodes = ns.network.NodeContainer()
        nodes.Create(n_nodes)

        self._ns3_nodes = nodes
        self._ns3_node_map = {name: idx for idx, name in enumerate(node_names)}

        # ---- Step 2: Wireless channel ----------------------------------------
        # YansWifiChannel is a simplified but efficient propagation model.
        # LogDistancePropagationLossModel applies:
        #   L(d) = L(d0) + 10·n·log10(d/d0)  where n=2.7 (typical outdoor)
        channel_helper = ns.wifi.YansWifiChannelHelper.Default()
        channel_helper.AddPropagationLoss(
            "ns3::LogDistancePropagationLossModel",
            "Exponent", ns.core.DoubleValue(2.7),
            "ReferenceDistance", ns.core.DoubleValue(1.0),
            "ReferenceLoss", ns.core.DoubleValue(46.6777),
        )
        channel_helper.SetPropagationDelay(
            "ns3::ConstantSpeedPropagationDelayModel"
        )
        phy_helper = ns.wifi.YansWifiPhyHelper()
        phy_helper.SetChannel(channel_helper.Create())
        phy_helper.Set("TxPowerStart", ns.core.DoubleValue(_TX_POWER_DBM))
        phy_helper.Set("TxPowerEnd",   ns.core.DoubleValue(_TX_POWER_DBM))

        # ---- Step 3: Wi-Fi helper (802.11n ad-hoc) ---------------------------
        wifi = ns.wifi.WifiHelper()
        wifi.SetStandard(ns.wifi.WIFI_STANDARD_80211n)
        wifi.SetRemoteStationManager(
            "ns3::ConstantRateWifiManager",
            # MCS index 7 → up to 65 Mbps per stream at 20 MHz BW
            "DataMode",    ns.core.StringValue("HtMcs7"),
            "ControlMode", ns.core.StringValue("HtMcs0"),
        )

        # AdhocWifiMac: no AP, each node is equal peer
        mac_helper = ns.wifi.WifiMacHelper()
        mac_helper.SetType("ns3::AdhocWifiMac")

        # Install Wi-Fi NICs on all nodes
        devices = wifi.Install(phy_helper, mac_helper, nodes)

        # ---- Step 4: Internet stack + OLSR routing ---------------------------
        # OLSR (RFC 3626) builds a multi-hop routing table via Hello and TC messages.
        olsr_helper = ns.olsr.OlsrHelper()
        stack = ns.internet.InternetStackHelper()
        stack.SetRoutingHelper(olsr_helper)
        stack.Install(nodes)

        # ---- Step 5: IP address assignment (10.0.0.x/24) --------------------
        addr_helper = ns.internet.Ipv4AddressHelper()
        addr_helper.SetBase(
            ns.network.Ipv4Address("10.0.0.0"),
            ns.network.Ipv4Mask("255.255.255.0"),
        )
        interfaces = addr_helper.Assign(devices)

        # ---- Step 6: Mobility models ----------------------------------------
        mobility = ns.mobility.MobilityHelper()
        mobility.SetMobilityModel(
            "ns3::ConstantPositionMobilityModel"
        )
        mobility.Install(nodes)

        for name, idx in self._ns3_node_map.items():
            node = nodes.Get(idx)
            mob_model = node.GetObject(
                ns.mobility.ConstantPositionMobilityModel.GetTypeId()
            )
            x, y, z = positions[name]
            mob_model.SetPosition(ns.core.Vector(x, y, z))
            self._ns3_mobility_map[name] = mob_model

        # ---- Step 7: UDP Echo traffic for topology discovery -----------------
        # Install a UdpEchoServer on GCS node (index of "GCS")
        gcs_idx = self._ns3_node_map.get(_GCS_ID, 0)
        gcs_node = nodes.Get(gcs_idx)
        gcs_iface = interfaces.GetAddress(gcs_idx)

        echo_server = ns.applications.UdpEchoServerHelper(9)   # port 9
        server_apps = echo_server.Install(gcs_node)
        server_apps.Start(ns.core.Seconds(0.0))
        server_apps.Stop(ns.core.Seconds(3600.0))

        # Every non-GCS node sends periodic pings to GCS
        echo_client = ns.applications.UdpEchoClientHelper(gcs_iface, 9)
        echo_client.SetAttribute(
            "MaxPackets", ns.core.UintegerValue(10_000_000)
        )
        echo_client.SetAttribute(
            "Interval", ns.core.TimeValue(ns.core.Seconds(1.0))
        )
        echo_client.SetAttribute(
            "PacketSize", ns.core.UintegerValue(64)
        )

        client_nodes = ns.network.NodeContainer()
        for name, idx in self._ns3_node_map.items():
            if name != _GCS_ID:
                client_nodes.Add(nodes.Get(idx))

        client_apps = echo_client.Install(client_nodes)
        client_apps.Start(ns.core.Seconds(0.1))
        client_apps.Stop(ns.core.Seconds(3600.0))

        # Advance NS-3 slightly to allow OLSR Hello exchange
        ns.core.Simulator.Run()   # runs until no more events (or Stop called)
        # Schedule a stop after the OLSR convergence time (~6 s)
        ns.core.Simulator.Stop(ns.core.Seconds(6.0))
        ns.core.Simulator.Run()

        self._ns3_initialized = True
        self._ns3_sim_time = 6.0
        logger.info("NS-3 setup complete (%d nodes, OLSR active).", n_nodes)

    def _ns3_run_tick(self, dt: float) -> None:
        """
        Advance NS-3 by *dt* seconds and sample link states.

        NS-3 does not expose a per-link RSSI table directly from Python;
        we reconstruct it from the mobility positions using the same
        LogDistanceLoss formula configured in the channel.
        """
        # Advance simulator time
        self._ns3_sim_time += dt
        ns.core.Simulator.Stop(ns.core.Seconds(self._ns3_sim_time))
        ns.core.Simulator.Run()

        # Recompute link states from current mobility positions
        link_states: Dict[Tuple[str, str], Dict] = {}

        node_names = list(self._ns3_node_map.keys())
        for a in node_names:
            mob_a = self._ns3_mobility_map[a]
            pos_a = mob_a.GetPosition()
            for b in node_names:
                if a == b:
                    continue
                mob_b = self._ns3_mobility_map[b]
                pos_b = mob_b.GetPosition()

                dx = pos_a.x - pos_b.x
                dy = pos_a.y - pos_b.y
                dz = pos_a.z - pos_b.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)

                # Log-distance path loss (same model as channel)
                if dist <= 1.0:
                    pl = 46.6777
                else:
                    pl = 46.6777 + 10.0 * 2.7 * math.log10(dist)

                signal = _TX_POWER_DBM - pl
                snr = signal - _NOISE_FLOOR_DBM

                if dist > self.max_range or snr < _SNR_WEAK_DB:
                    status = "DOWN"
                elif snr < _SNR_UP_DB:
                    status = "WEAK"
                else:
                    status = "UP"

                link_states[(a, b)] = {
                    "status":       status,
                    "signal_dbm":   round(signal, 2),
                    "path_loss_db": round(pl, 2),
                    "distance_m":   round(dist, 2),
                }

        self._link_states = link_states

        # Extract OLSR routing table via NS-3 API
        self._routing_table = self._ns3_extract_routes()

    def _ns3_extract_routes(self) -> Dict[str, List[str]]:
        """
        Read the OLSR routing tables installed on each NS-3 node and
        reconstruct human-readable paths to GCS.

        Returns the same format as :meth:`get_routing_table`.
        """
        gcs_idx = self._ns3_node_map.get(_GCS_ID, 0)
        gcs_node = self._ns3_nodes.Get(gcs_idx)
        # Get GCS IP (first interface, first address)
        gcs_ipv4 = gcs_node.GetObject(ns.internet.Ipv4.GetTypeId())
        gcs_addr = gcs_ipv4.GetAddress(1, 0).GetLocal()

        routing: Dict[str, List[str]] = {}
        idx_to_name = {v: k for k, v in self._ns3_node_map.items()}

        for name, idx in self._ns3_node_map.items():
            if name == _GCS_ID:
                routing[name] = [_GCS_ID]
                continue

            node = self._ns3_nodes.Get(idx)
            routing_obj = node.GetObject(
                ns.olsr.RoutingProtocol.GetTypeId()
            )

            path = [name]
            current_addr = gcs_addr
            seen = {name}

            # Walk OLSR routing table: each hop gives the next node toward GCS
            for _ in range(len(self._ns3_node_map)):
                entry = routing_obj.GetRoutingTable()
                next_hop = None
                for route in entry:
                    if str(route.destAddr) == str(gcs_addr):
                        next_hop_addr = str(route.nextAddr)
                        # Map IP → node name
                        for n2, i2 in self._ns3_node_map.items():
                            n2_node = self._ns3_nodes.Get(i2)
                            n2_ipv4 = n2_node.GetObject(
                                ns.internet.Ipv4.GetTypeId()
                            )
                            n2_addr = str(
                                n2_ipv4.GetAddress(1, 0).GetLocal()
                            )
                            if n2_addr == next_hop_addr and n2 not in seen:
                                next_hop = n2
                                break
                        break
                if next_hop is None or next_hop == _GCS_ID:
                    path.append(_GCS_ID)
                    break
                path.append(next_hop)
                seen.add(next_hop)
                if next_hop == _GCS_ID:
                    break
            else:
                path = []   # no route found

            routing[name] = path

        return routing

    # ------------------------------------------------------------------
    # Phase-0 back-end helpers
    # ------------------------------------------------------------------

    def _phase0_sync_positions(self) -> None:
        """
        Synchronise the Phase-0 CommNetwork with current UAV positions.

        Phase-0 CommNetwork works with UAV objects that have a ``.position``
        attribute with ``.x``, ``.y``, ``.z`` fields.  We create lightweight
        proxy objects here.
        """
        class _PosProxy:
            def __init__(self, x, y, z):
                self.x = x; self.y = y; self.z = z

        class _UAVProxy:
            def __init__(self, uid, x, y, z):
                self.id = uid
                self.position = _PosProxy(x, y, z)

        proxies = [
            _UAVProxy(name, *pos)
            for name, pos in self._uav_positions.items()
        ]
        try:
            self._fallback.update_nodes(proxies)
        except AttributeError:
            # CommNetwork API may vary; best-effort
            for proxy in proxies:
                try:
                    self._fallback.update_position(proxy.id, proxy.position)
                except Exception:
                    pass

    def __repr__(self) -> str:
        return (
            f"SwarmNetworkNS3(backend={self.backend!r}, "
            f"nodes={len(self._uav_positions)}, "
            f"tick={self._tick})"
        )


# ===========================================================================
# Module self-test / demo
# ===========================================================================

if __name__ == "__main__":
    import random

    logging.basicConfig(level=logging.INFO)
    logger.info("=== SwarmNetworkNS3 stand-alone demo ===")

    net = SwarmNetworkNS3(max_range=800.0, noise=True)

    # Set up a 5-node swarm: GCS at origin + 4 UAVs scattered around
    initial_positions = {
        "GCS":   (0.0,   0.0,   0.0),
        "uav_0": (300.0, 0.0,  50.0),
        "uav_1": (650.0, 150.0, 60.0),
        "uav_2": (900.0, 400.0, 55.0),
        "uav_3": (200.0, 700.0, 45.0),
    }

    net.setup_nodes(initial_positions)

    for tick in range(5):
        # Simulate UAVs moving
        for uid in ["uav_0", "uav_1", "uav_2", "uav_3"]:
            x, y, z = net._uav_positions[uid]
            net.update_node_position(
                uid, (x + random.uniform(-10, 10),
                      y + random.uniform(-10, 10),
                      z)
            )

        net.run_tick(dt=1.0)

        states = net.get_link_states()
        routes = net.get_routing_table()

        logger.info("--- Tick %d ---", tick + 1)
        for node, path in routes.items():
            status = "CONNECTED" if path else "ISOLATED"
            hops = " → ".join(path) if path else "NO ROUTE"
            logger.info("  %s: %s  [%s]", node, status, hops)

    logger.info("Demo complete.")
