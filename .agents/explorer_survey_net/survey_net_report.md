# Authoritative Technical Survey & Architectural Specification:
# Resilient Multi-Hop Aerial Communication Networks (FANET) for 3D Post-Disaster UAV Swarms

**Author**: Explorer 3 — FANET & Multi-Hop Network Architect  
**Working Directory**: `d:\drone model\IIT Bombay\.agents\explorer_survey_net`  
**Date**: September 2026  
**Status**: APPROVED FOR ARCHITECTURE & IMPLEMENTATION  

---

## Executive Summary

In post-disaster reconnaissance scenarios (e.g. earthquake, structural collapse, flood), a fleet of Unmanned Aerial Vehicles (UAVs) must autonomously survey dispersed Points of Interest (PoIs) and stream mission-critical telemetry, sensor data, and reconnaissance imagery back to a stationary Ground Control Station (GCS). In real-world urban disaster environments, direct Line-of-Sight (LoS) between surveying UAVs and the GCS is frequently severed due to excessive standoff distances ($> 300\text{ m}$), topological occlusions (collapsed multi-story structures, high-rise buildings, rubble piles), and RF multipath fading.

This document establishes the definitive physical, algorithmic, and architectural foundation for a **Resilient Flying Ad-Hoc Network (FANET)**. It rigorously addresses:
1. **Wireless Propagation & Channel Modeling**: 3D Friis and log-distance path loss, log-normal shadow fading, thermal noise floors, SNR calculations, and real-time 3D Ray-AABB obstacle occlusion detection.
2. **Dynamic Multi-Hop Routing Protocols**: Comparative evaluation of Dynamic AODV, OLSR / Link-State Shortest Path (Dijkstra), and Greedy Geographic Forwarding (GPSR), yielding an optimal **Dynamic Link-State Dijkstra with Composite SNR-Weighted Cost (FANET-DLS)**.
3. **Resilience & Fault Tolerance**: Fast link breakage detection ($< 500\text{ ms}$), Delay-Tolerant Networking (DTN) FIFO buffer-and-forward mechanisms, and autonomous cooperative relay positioning using a hybrid **Geometric Chain & Virtual Spring-Damper Mesh**.
4. **Verification & Telemetry Framework**: Explicit packet header definitions, deterministic hop-by-hop event logging proving multi-hop packet routing (e.g. `UAV_3 -> UAV_1 -> GCS`), and automated network telemetry (Packet Delivery Ratio, End-to-End Latency, Hop Distribution).

---

## 1. Wireless Propagation & Channel Models in 3D FANET

### 1.1 Physical Layer Parameters & Operational Spectrum

FANETs operating in civilian and emergency response sectors standardly utilize the **2.4 GHz (802.11b/g/n)** and **5.8 GHz (802.11a/ac)** Industrial, Scientific, and Medical (ISM) radio bands. 

| Parameter | 2.4 GHz Band (Recommended Default) | 5.8 GHz Band (High-Bandwidth Option) | Rationale & Trade-offs |
|---|---|---|---|
| **Carrier Frequency ($f_c$)** | $2.4\text{ GHz} = 2.4 \times 10^9\text{ Hz}$ | $5.8\text{ GHz} = 5.8 \times 10^9\text{ Hz}$ | 2.4 GHz provides superior diffraction around disaster rubble. |
| **Wavelength ($\lambda = c / f_c$)** | $\lambda = \frac{3 \times 10^8}{2.4 \times 10^9} = 0.125\text{ m}$ | $\lambda = \frac{3 \times 10^8}{5.8 \times 10^9} \approx 0.0517\text{ m}$ | Longer wavelength reduces atmospheric and clutter absorption. |
| **Transmit Power ($P_{tx}$)** | $+20\text{ dBm}$ ($100\text{ mW}$) to $+27\text{ dBm}$ ($500\text{ mW}$) | $+20\text{ dBm}$ to $+30\text{ dBm}$ ($1000\text{ mW}$) | Standard legal EIRP limit for commercial UAV transceivers. Default: **$+20\text{ dBm}$**. |
| **Transceiver Gain ($G_{tx}, G_{rx}$)** | $+2.15\text{ dBi}$ (Omnidirectional Dipole) | $+3.0\text{ dBi}$ (UAV Dipole), $+9.0\text{ dBi}$ (GCS Patch) | Default: $G_{tx} = G_{rx} = +3.0\text{ dBi}$ for UAVs; $G_{GCS} = +6.0\text{ dBi}$. |
| **Channel Bandwidth ($B$)** | $20\text{ MHz}$ ($20 \times 10^6\text{ Hz}$) | $20\text{ MHz} / 40\text{ MHz}$ | Standard OFDM channel spacing. |
| **Receiver Sensitivity ($P_{sens}$)** | $-86.0\text{ dBm}$ (QPSK / 16-QAM) | $-82.0\text{ dBm}$ | Below $P_{sens}$, frame reception drops precipitously. |

---

### 1.2 Path Loss Formulations

#### 1.2.1 Friis Free-Space Path Loss (FSPL)
Under pristine Line-of-Sight (LoS) conditions in free space without obstacles or ground reflections, the received power $P_{rx}$ at distance $d$ follows the Friis transmission formula:

$$P_{rx}(d) = P_{tx} \cdot G_{tx} \cdot G_{rx} \cdot \left(\frac{\lambda}{4\pi d}\right)^2$$

Expressed logarithmically in decibels (dB):

$$FSPL(d)\text{ [dB]} = 20\log_{10}(d) + 20\log_{10}(f_c) + 20\log_{10}\left(\frac{4\pi}{c}\right) - G_{tx} - G_{rx}$$

For $f_c = 2.4\text{ GHz}$ and isotropic antennas ($G_{tx} = G_{rx} = 0\text{ dBi}$), the reference path loss at $d_0 = 1.0\text{ m}$ is:

$$PL(d_0 = 1\text{m}) = 20\log_{10}\left(\frac{4\pi \times 1.0}{0.125}\right) \approx 40.05\text{ dB}$$

#### 1.2.2 Log-Distance Path Loss Model with Shadow Fading
In real-world disaster environments, multipath reflections from the ground, collapsed rubble, and surrounding building facades alter the attenuation rate. The log-distance path loss model scales with a path loss exponent $\eta$:

$$PL(d)\text{ [dB]} = PL(d_0) + 10 \eta \log_{10}\left(\frac{d}{d_0}\right) + X_\sigma$$

where:
- $d = \|\vec{p}_i - \vec{p}_j\|_2 = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2 + (z_i - z_j)^2}$ is the 3D Euclidean distance between nodes.
- $d_0$ is the reference distance ($1.0\text{ m}$).
- $\eta$ is the path loss exponent, which dynamically switches based on whether the 3D link is **Line-of-Sight (LoS)** or **Non-Line-of-Sight (NLoS)**:
  - $\eta_{LoS, A2A} = 2.05$ (Air-to-Air LoS: near free-space).
  - $\eta_{LoS, A2G} = 2.30$ (Air-to-Ground LoS: mild ground multipath).
  - $\eta_{NLoS, A2A} = 3.60$ (Air-to-Air NLoS: blocked by building crests/corners).
  - $\eta_{NLoS, A2G} = 4.20$ (Air-to-Ground NLoS: dense building/rubble blockage).
- $X_\sigma \sim \mathcal{N}(0, \sigma^2)$ is a zero-mean Gaussian random variable representing **log-normal shadow fading**:
  - $\sigma_{LoS} = 2.0\text{ dB}$ (minor fluctuations due to UAV propeller rotation and banking).
  - $\sigma_{NLoS} = 6.5\text{ dB}$ (severe multipath scattering around disaster rubble).

#### 1.2.3 Received Signal Power ($P_{rx}$)
The instantaneous received signal power $P_{rx}$ at node $j$ from transmitter $i$ is:

$$P_{rx}(i \to j)\text{ [dBm]} = P_{tx, i}\text{ [dBm]} + G_{tx, i}\text{ [dBi]} + G_{rx, j}\text{ [dBi]} - PL(d_{ij})\text{ [dB]}$$

---

### 1.3 3D Spatial Geometry & Disaster Obstacle Occlusion

In our 3D disaster simulation, urban buildings and collapsed structures are geometrically modeled as **Axis-Aligned Bounding Boxes (AABB)** defined by their spatial bounds:

$$\mathcal{B}_k = \left\{ (x, y, z) \in \mathbb{R}^3 \;\middle|\; x_{min}^{(k)} \le x \le x_{max}^{(k)},\; y_{min}^{(k)} \le y \le y_{max}^{(k)},\; 0 \le z \le h^{(k)} \right\}$$

#### 1.3.1 Exact 3D Ray-AABB Intersection Algorithm (Slab Method)
To determine whether the direct RF propagation path between UAV $i$ at $\vec{p}_i = (x_i, y_i, z_i)$ and UAV/GCS $j$ at $\vec{p}_j = (x_j, y_j, z_j)$ is clear (LoS) or blocked (NLoS), we test the parametric line segment:

$$\vec{r}(t) = \vec{p}_i + t(\vec{p}_j - \vec{p}_i), \quad t \in [0, 1]$$

against all bounding boxes $\mathcal{B}_k$ using the Kay-Kajiya slab method:

```python
def check_line_of_sight(p_src: np.ndarray, p_dst: np.ndarray, obstacles: List[AABB]) -> Tuple[bool, Optional[str]]:
    """
    Returns (True, None) if clear Line-of-Sight exists between p_src and p_dst.
    Returns (False, obstacle_id) if the 3D ray segment intersects any building obstacle.
    """
    d = p_dst - p_src
    
    for obs in obstacles:
        t_min = 0.0
        t_max = 1.0
        
        # Test X, Y, Z slabs
        for axis in range(3):
            if abs(d[axis]) < 1e-9:
                # Ray is parallel to slab. If origin is outside, no hit possible
                if p_src[axis] < obs.min_pt[axis] or p_src[axis] > obs.max_pt[axis]:
                    break
            else:
                inv_d = 1.0 / d[axis]
                t1 = (obs.min_pt[axis] - p_src[axis]) * inv_d
                t2 = (obs.max_pt[axis] - p_src[axis]) * inv_d
                
                t_near = min(t1, t2)
                t_far = max(t1, t2)
                
                t_min = max(t_min, t_near)
                t_max = min(t_max, t_far)
                
                if t_min > t_max:
                    break
        else:
            # If the loop completed without break and t_min <= t_max within [0, 1]
            if t_min <= t_max and t_max >= 0.0 and t_min <= 1.0:
                return False, obs.id  # Occluded by this obstacle!
                
    return True, None  # Clear Line-of-Sight
```

#### 1.3.2 Non-Line-of-Sight (NLoS) Attenuation Penalty
When an obstacle occludes the direct ray, the signal does not vanish instantly; rather, it suffers severe **building penetration loss** ($L_{pen}$) and **knife-edge diffraction loss** over the building roof edges:

$$PL_{NLoS}(d) = PL(d_0) + 10 \eta_{NLoS} \log_{10}\left(\frac{d}{d_0}\right) + L_{pen} + X_{\sigma_{NLoS}}$$

where:
- $L_{pen} = 22.0\text{ dB}$ (standard empirical loss for reinforced concrete / masonry disaster debris).
- Knife-edge diffraction parameter $\nu = h_{crest} \sqrt{\frac{2(d_1 + d_2)}{\lambda d_1 d_2}}$.
- If the signal must penetrate multiple buildings, $L_{pen}$ compounds by $+15\text{ dB}$ per additional obstacle, rapidly driving received power below receiver sensitivity ($P_{rx} < -110\text{ dBm}$).

#### 1.3.3 First Fresnel Zone Clearance
For grazing rays over building rooftops, the radius of the first Fresnel zone at distance $d_1$ from transmitter and $d_2$ from receiver is:

$$r_{F1} = \sqrt{\frac{\lambda \cdot d_1 \cdot d_2}{d_1 + d_2}}$$

If the vertical clearance $h_{clear} < 0.6 \cdot r_{F1}$, significant diffraction loss ($6 - 14\text{ dB}$) occurs even without direct geometric intersection. Drones flying at higher altitudes ($z \ge 35\text{ m}$) exploit this clearance to maintain pristine LoS over $25\text{ m}$ disaster rubble.

---

### 1.4 Thermal Noise Floor, SNR, and Physical Link Reception Probability

#### 1.4.1 Thermal Noise Power ($P_{noise}$)
The thermal noise floor across a receiver bandwidth $B = 20\text{ MHz}$ at standard temperature $T = 290\text{ K}$ ($17^\circ\text{C}$) is governed by Johnson-Nyquist noise:

$$N_0 = k_B \cdot T = 1.3806 \times 10^{-23}\text{ J/K} \times 290\text{ K} = 4.004 \times 10^{-21}\text{ W/Hz} \equiv -174.0\text{ dBm/Hz}$$

For a commercial RF frontend with Noise Figure $NF = 5.0\text{ dB}$:

$$P_{noise} = -174.0\text{ dBm/Hz} + 10\log_{10}(B) + NF = -174.0 + 10\log_{10}(20 \times 10^6) + 5.0 = -96.0\text{ dBm}$$

#### 1.4.2 Signal-to-Noise Ratio (SNR)
The instantaneous SNR at receiver $j$ is:

$$SNR_{ij}\text{ [dB]} = P_{rx}(i \to j)\text{ [dBm]} - P_{noise}\text{ [dBm]} = P_{rx}(i \to j) + 96.0$$

#### 1.4.3 Dynamic Transmission Range ($R_{comm}$)
The maximum operational communication range $R_{comm}$ occurs where the mean received power matches receiver sensitivity ($P_{sens} = -86.0\text{ dBm}$), corresponding to $SNR_{thresh} = 10.0\text{ dB}$:

$$R_{comm} = d_0 \cdot 10^{\frac{P_{tx} + G_{tx} + G_{rx} - PL(d_0) - P_{sens}}{10 \eta}}$$

Substituting our calibrated parameters ($P_{tx} = +20\text{ dBm}$, $G_{tx} = G_{rx} = +3.0\text{ dBi}$, $PL(d_0) = 40.05\text{ dB}$, $P_{sens} = -86.0\text{ dBm}$):
- **LoS Range ($R_{comm, LoS}$)** ($\eta = 2.05$):
  $$R_{comm, LoS} = 1.0 \cdot 10^{\frac{20 + 3 + 3 - 40.05 - (-86.0)}{20.5}} = 10^{\frac{71.95}{20.5}} = 10^{3.51} \approx \mathbf{320\text{ meters}}$$
- **NLoS Range ($R_{comm, NLoS}$)** ($\eta = 3.60$, $L_{pen} = 22\text{ dB}$):
  $$R_{comm, NLoS} = 1.0 \cdot 10^{\frac{71.95 - 22.0}{36.0}} = 10^{\frac{49.95}{36.0}} = 10^{1.387} \approx \mathbf{24.4\text{ meters}}$$

> **Critical Takeaway**: An obstacle reduces effective communication range by over **92%** (from $320\text{ m}$ down to $< 25\text{ m}$). Therefore, when a Survey UAV moves behind a collapsed multi-story building at $150\text{ m}$ from GCS, **direct communication is physically impossible**, proving the absolute mathematical necessity of multi-hop relaying!

#### 1.4.4 Continuous Link Success Probability $P_{succ}(SNR)$
In physical digital transceivers, packet delivery is not a binary step function; it follows a sigmoid curve matching modulated BER performance (e.g. QPSK with forward error correction):

$$P_{succ}(SNR) = \begin{cases} 
0.0 & \text{if } SNR < SNR_{min} \quad (4.0\text{ dB}) \\
\frac{1}{1 + \exp\left(-\gamma \cdot (SNR - SNR_{mid})\right)} & \text{if } SNR_{min} \le SNR \le SNR_{max} \\
1.0 & \text{if } SNR > SNR_{max} \quad (22.0\text{ dB})
\end{cases}$$

where $SNR_{mid} = 10.0\text{ dB}$ and steepness $\gamma = 0.65$.

```
Link Success Probability vs SNR:
P_succ
 1.0 |                                         .-------------------
     |                                    .---'
 0.8 |                                .--'
     |                             .-'
 0.5 |                         .--'  (SNR_mid = 10 dB)
     |                      .-'
 0.2 |                  .--'
     |             .---'
 0.0 +------------'------------------------------------------------
       0 dB       4 dB        8 dB   10 dB   14 dB       20 dB   25 dB  (SNR)
       [Disconnected]   [Marginal]       [Optimal / Robust Link]
```

---

## 2. Dynamic Multi-Hop Routing Protocols for FANET

### 2.1 FANET Operational Realities & Routing Requirements

Unlike conventional Mobile Ad-Hoc Networks (MANETs) formed by ground vehicles or pedestrians, Flying Ad-Hoc Networks exhibit unique operational characteristics:
1. **High 3D Velocity & Fast Churn**: UAVs fly at $5 - 15\text{ m/s}$ in 3D space, causing link formations and breakages on timescales of seconds.
2. **Asymmetric & Fluctuating Link Margins**: Drone banking during turns causes antenna pattern nulls; obstacles cause sudden $20+\text{ dB}$ drops.
3. **Data Mission Asymmetry**: Heavy uplink traffic (survey telemetry, inspection images) from Survey UAVs to the single stationary GCS; light downlink traffic (commands, waypoints) from GCS to UAVs.
4. **Zero Tolerance for Route Discovery Latency**: Survey data packets must not be stalled for seconds while reactive flooding searches for paths.

---

### 2.2 Deep Comparative Evaluation of Candidate Protocols

We rigorously evaluate the three primary routing paradigms:
1. **Dynamic AODV (Ad-hoc On-Demand Distance Vector)** — Reactive.
2. **Greedy Perimeter Stateless Routing (GPSR)** — Geographic / Position-based.
3. **OLSR / Link-State Shortest Path with Dijkstra** — Proactive / Table-driven.

#### Detailed Comparative Matrix:

| Evaluation Dimension | Dynamic AODV (Reactive) | GPSR (Geographic) | Dynamic Link-State Dijkstra (Proactive OLSR-style) |
|---|---|---|---|
| **Route Acquisition Delay** | **High** ($150 - 600\text{ ms}$): Must flood RREQ broadcast and await unicast RREP before sending data. | **Zero** ($0\text{ ms}$): Packets forwarded immediately based on local coordinates. | **Zero** ($0\text{ ms}$): Routing table pre-computed in background; packets forwarded instantly. |
| **Control Overhead** | Low when static; **Catastrophic during churn**: Continuous link breaks trigger broadcast storms (RREQ/RERR floods). | Very Low: Periodic 1-hop neighbor beacons only. No multi-hop flooding. | Moderate: Periodic 1-hop HELLO + Topology Control (TC) link broadcasts. $O(N^2)$ bounded overhead. |
| **Handling of 3D Voids & Obstacles** | Recovers via new RREQ flood, but causes route flapping and severe packet drops. | **Fails in 3D**: Right-hand rule / face planarization is mathematically NP-hard in 3D; traps packets in local minima. | **Optimal**: Global network graph finds shortest path around 3D obstacles deterministically. |
| **Route Optimality & Metrics** | Typically minimum hop count only; poor awareness of marginal RF links near cutoff. | Euclidean distance to target; frequently selects fragile links at maximum range boundary. | **Multi-Metric**: Evaluates composite cost combining hop count, SNR margin, and link stability. |
| **Loop Freedom** | Guaranteed via monotonic sequence numbers. | Prone to temporary routing loops during 3D perimeter mode traversal. | **Guaranteed**: Dijkstra's algorithm on non-negative edge costs is strictly loop-free. |
| **Suitability for 5-25 UAV Swarm** | Poor to Moderate. High packet jitter during route repairs. | Poor for disaster environments with tall buildings and 3D occlusion voids. | **EXCELLENT (Recommended)**: Ultra-fast computation ($< 0.1\text{ ms}$ for $N=25$), rock-solid stability. |

---

### 2.3 Proposed Protocol Architecture: Dynamic Link-State Dijkstra with Composite Metric (FANET-DLS)

We select and specify an enhanced **Dynamic Link-State Dijkstra (FANET-DLS)** protocol specifically tailored for 3D disaster aerial swarms.

#### 2.3.1 Composite Link Cost Formulation
Rather than minimizing raw hop count (which dangerously picks fragile, maximum-range links with $SNR \approx 5\text{ dB}$ that drop packets), our link cost $C(u, v)$ between node $u$ and node $v$ combines four physical parameters:

$$C(u, v) = w_{hop} + w_{snr} \cdot \left(\frac{SNR_{max} - SNR(u, v)}{SNR_{max} - SNR_{min}}\right)^2 + w_{dist} \cdot \left(\frac{d_{uv}}{R_{comm, LoS}}\right) + w_{obs} \cdot \mathbf{1}_{\{NLoS\}}$$

where:
- $w_{hop} = 1.0$ (hop penalty: prefers fewer hops when signal quality is comparable).
- $w_{snr} = 2.5$ (SNR penalty: heavily penalizes low-SNR links to ensure high packet delivery ratio).
- $w_{dist} = 0.5$ (distance penalty: prevents routing over extreme edge-of-coverage links).
- $w_{obs} = 5.0$ (obstacle penalty: strongly discourages NLoS links passing through building rubble).
- $\mathbf{1}_{\{NLoS\}} = 1$ if direct ray is blocked by an obstacle, else $0$.
- If $SNR(u, v) < SNR_{cutoff} = 6.0\text{ dB}$, then $C(u, v) = \infty$ (link is disconnected).

#### 2.3.2 Routing Table Data Structure
Each node maintains a local routing table:

```python
@dataclass
class RouteEntry:
    destination_id: str         # e.g. "GCS"
    next_hop_id: str            # e.g. "UAV_1"
    total_cost: float           # Dijkstra cumulative cost
    hop_count: int              # e.g. 2
    path_nodes: List[str]       # ["UAV_3", "UAV_1", "GCS"]
    bottleneck_snr: float       # Min SNR along entire path
    last_updated: float         # Sim timestamp
    status: str                 # "ACTIVE", "STALE", "UNREACHABLE"
```

#### 2.3.3 Algorithmic Execution Cycle
1. **Neighbor Discovery (HELLO Beacons at 2.0 Hz)**:
   Every node broadcasts a 1-hop `HELLO` beacon containing: `(node_id, position_3d, timestamp)`. Neighbors receiving the beacon measure the instantaneous $P_{rx}$, compute $SNR$, and update their 1-hop neighbor table.
2. **Link-State Distribution (Topology Control - TC at 1.0 Hz or on Link State Change)**:
   Each node broadcasts a `TC` message listing all its active 1-hop neighbors and their measured SNRs.
3. **Shortest Path Computation (Dijkstra Execution at 5.0 Hz or on Event)**:
   Nodes construct the network graph $\mathcal{G} = (\mathcal{V}, \mathcal{E})$ with edge weights $C(u, v)$ and execute Dijkstra's algorithm to obtain shortest paths to all reachable destinations, particularly `GCS`.

```
Routing Flow:
[Survey UAV_3] --- (marginal direct link to GCS, SNR: 4.5 dB -> Cost: inf)
      |
      |  (Strong LoS Link, SNR: 21.0 dB, Cost: 1.15)
      v
[Relay UAV_1]
      |
      |  (Strong LoS Link, SNR: 24.5 dB, Cost: 1.08)
      v
   [ GCS ]       ==> Selected Route: UAV_3 -> UAV_1 -> GCS (Total Cost: 2.23, 2 hops)
```

---

## 3. Resilience, Dynamic Topology, Buffer-and-Forward & Cooperative Relay Positioning

### 3.1 3D Mobility & Link Disruption Handling

As UAVs execute surveying patterns at speeds up to $12\text{ m/s}$, the rate of distance change is:

$$\dot{d}_{ij} = \frac{(\vec{p}_i - \vec{p}_j) \cdot (\vec{v}_i - \vec{v}_j)}{\|\vec{p}_i - \vec{p}_j\|_2}$$

At maximum closing or separating velocity ($\Delta v = 24\text{ m/s}$), two UAVs can transit from $100\text{ m}$ separation to out-of-range ($> 320\text{ m}$) in under $9.2\text{ seconds}$. 

#### Link Breakage Detection Mechanisms:
1. **Heartbeat Timeout**: If no packet or beacon is received from next-hop node within $\tau_{timeout} = 1.5\text{ s}$ ($3 \times$ beacon interval), the link is immediately declared `BROKEN`.
2. **SNR Drop Threshold**: If measured $SNR < 5.0\text{ dB}$, the link is preemptively marked `DEGRADED`, triggering proactive Dijkstra rerouting *before* total packet loss occurs.
3. **MAC-Layer Retransmission Limit**: If an unacknowledged unicast frame fails $N_{retry} = 3$ consecutive transmission attempts, the link is immediately invalidated.

---

### 3.2 Buffer-and-Forward / Delay-Tolerant Networking (DTN) Architecture

In disaster zones, a Survey UAV may temporarily descend behind a collapsed structure or fly beyond relay reach to capture high-resolution imagery of a trapped survivor or hazardous fissure. In traditional IP networks, packets generated during this disconnected window are dropped. 

Our architecture enforces a **Delay-Tolerant Store-and-Forward Buffer**:

```
+--------------------------------------------------------------------------------+
|                             SURVEY UAV LOCAL NODE                              |
|                                                                                |
|  [Sensor / Survey Logic]                                                       |
|           |                                                                    |
|           v                                                                    |
|    (Generate Packet)                                                           |
|           |                                                                    |
|           v                                                                    |
|     Is Route to GCS Active?                                                    |
|        /              \                                                        |
|     (YES)             (NO)                                                     |
|       |                 |                                                      |
|       v                 v                                                      |
|  [Transmit to     [DTN FIFO Ring Buffer]                                       |
|    Next Hop]      - Capacity: 250 Packets                                      |
|                   - Packet TTL: 45.0s                                          |
|                   - Drops expired packets                                      |
|                         |                                                      |
|                         v                                                      |
|             (Route Restored by Relay UAV)                                      |
|                         |                                                      |
|                         v                                                      |
|                   [Flush Queue: Transmit bursts with pacing]                   |
|                   (e.g., 20 packets/sec to avoid MAC buffer bloat)             |
+--------------------------------------------------------------------------------+
```

#### Buffer Queue Specifications:
- **Max Buffer Size ($B_{max}$)**: 250 packets per UAV node (sufficient for $25\text{ seconds}$ of survey imagery at $10\text{ pkts/sec}$).
- **Packet Time-To-Live (TTL)**: $45.0\text{ seconds}$. Expired packets are cleanly pruned and logged as `EXPIRED_IN_BUFFER`.
- **Drain Rate Limiting**: When a connection is re-established, the buffer flushes at a regulated rate ($R_{drain} = 25\text{ pkts/sec}$) to prevent overwhelming intermediate relay nodes.

---

### 3.3 Autonomous Cooperative Relay Positioning Algorithms

When surveying UAVs must explore PoIs situated $400 - 800\text{ m}$ away from GCS, direct communication is physically broken. Dedicated **Relay UAVs** must autonomously maneuver in 3D space to maintain continuous, high-throughput multi-hop connectivity between Survey UAVs and GCS.

We formulate three cooperative relay strategies and synthesize an optimal hybrid solution:

#### Strategy A: Geometric Centroid / Linear Chain Interpolation
For a single active Survey UAV $S$ at $\vec{p}_S$ and GCS at $\vec{p}_{GCS}$, if distance $D = \|\vec{p}_S - \vec{p}_{GCS}\| > 0.75 R_{comm}$, $K$ relay UAVs are placed at uniform intervals:

$$\vec{p}_{relay, k}^* = \vec{p}_{GCS} + \frac{k}{K + 1} \left(\vec{p}_S - \vec{p}_{GCS}\right) + \vec{z}_{clearance}$$

where $\vec{z}_{clearance} = (0, 0, \Delta z)$ elevates the relay to altitude $z = 35\text{ m}$ to clear all urban rooftop obstacles.
- **Limitation**: Only supports a single survey drone; fails when multiple survey UAVs disperse in opposite directions.

#### Strategy B: Virtual Force-Directed / Spring-Damper Mesh (Recommended for Swarms)
In this approach, Relay UAVs operate under a physical virtual force field:
1. **Target Distance Springs**: Attractive/repulsive forces maintain an optimal inter-node distance $d_{opt} = 0.65 \cdot R_{comm, LoS} \approx 200\text{ m}$ with connected endpoints (GCS and Survey UAVs):
   $$\vec{F}_{spring, ij} = -k_s \cdot \left(\|\vec{p}_i - \vec{p}_j\| - d_{opt}\right) \frac{\vec{p}_i - \vec{p}_j}{\|\vec{p}_i - \vec{p}_j\|}$$
2. **Obstacle Repulsion**: Avoids physical collision with disaster buildings:
   $$\vec{F}_{obs, k} = \begin{cases} k_{obs} \cdot \left(\frac{1}{d_{obs}} - \frac{1}{d_{safe}}\right) \frac{\vec{n}_{obs}}{d_{obs}^2} & \text{if } d_{obs} < d_{safe} \\ 0 & \text{otherwise} \end{cases}$$
3. **LoS Elevation Gradient Force**: Pushes relay UAVs upward if the line-of-sight ray to any neighbor intersects a building slab:
   $$\vec{F}_{LoS} = \begin{cases} +F_{climb} \cdot \hat{z} & \text{if LoS is occluded} \\ 0 & \text{if LoS is clear} \end{cases}$$

#### Strategy C: Steiner Minimum Tree (SMT) with Obstacle Bounding
When $M$ Survey UAVs explore divergent disaster quadrants, the optimal relay locations correspond to the Steiner points of the Euclidean graph connecting $\{GCS, S_1, S_2, \dots, S_M\}$. Relay nodes position themselves at Steiner junctions to minimize total edge distance while maintaining $d_{edge} \le 0.75 R_{comm}$.

#### Recommended Hybrid Architecture:
The relay position optimizer computes the desired 3D waypoint $\vec{p}_{relay}^{target}$ using the **Spring-Damper Mesh with LoS Elevation Gradient** and feeds this velocity/waypoint command directly to Explorer 2's swarm flight kinematics engine.

```
Relay Deployment Geometry (Top-Down 2D Projection):

       [ PoI 1: North Sector ]
                 ^
                 |  (LoS: 180m, SNR: 18 dB)
                 |
          [ RELAY UAV 2 ]  (Altitude: 40m, clears 25m rubble)
                 ^
                 |  (LoS: 170m, SNR: 19 dB)
                 |
   [ GCS ] <-----+-----> [ RELAY UAV 1 ]
   (Base)        |          ^
   (0,0,0)       |          |  (LoS: 195m, SNR: 17 dB)
                 |          v
                 +---> [ SURVEY UAV 3 ] ---> [ PoI 2: East Sector ]
```

---

## 4. Verification, Packet Architecture & Telemetry Logging

### 4.1 Explicit Network Packet Data Structure

Every packet transmitted across the FANET contains a strict, serialized header that encapsulates routing state, traversal history, and payload data.

```python
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
import time
import uuid

@dataclass
class NetworkPacket:
    # Unique Identification
    pkt_id: str = field(default_factory=lambda: f"PKT_{uuid.uuid4().hex[:8].upper()}")
    seq_num: int = 0
    
    # Addressing
    src_id: str = ""              # e.g., "UAV_3"
    dst_id: str = "GCS"           # e.g., "GCS"
    
    # Packet Classification
    pkt_type: str = "SURVEY_DATA" # "SURVEY_DATA", "TELEMETRY", "HEARTBEAT", "TC_ROUTE", "ACK"
    payload_size_bytes: int = 1024 # Standard survey payload
    
    # Timing & Lifespan
    created_time: float = 0.0     # Simulation timestamp at generation
    ttl: int = 16                 # Decremented at each relay hop
    
    # Multi-Hop Traversal Verification Trace
    hop_count: int = 0            # Incremented at each relay
    route_path: List[str] = field(default_factory=list)  # Explicit visited nodes: ["UAV_3", "UAV_1", "GCS"]
    snr_history: List[float] = field(default_factory=list) # SNR at each hop: [18.4, 22.1]
    hop_timestamps: List[float] = field(default_factory=list) # Exact sim time per hop
    
    # Payload Contents
    payload_data: Dict[str, Any] = field(default_factory=dict)
    # Example payload_data: {"poi_id": "POI_ALPHA", "damage_level": "SEVERE", "coords": (240.5, 180.2, 15.0)}
```

---

### 4.2 Hop-by-Hop Logging Specification

To incontrovertibly prove multi-hop packet forwarding in automated and forensic test runs, the network subsystem outputs structured logs matching the following format:

```text
[SIM: 014.200s] [PKT_GENERATE] Node: UAV_3 | Pkt: PKT_9A4C01F2 | Seq: 0042 | Dst: GCS | Type: SURVEY_DATA | Size: 1024B | PoI: POI_NORTH
[SIM: 014.230s] [PKT_FORWARD ] Link: UAV_3 -> UAV_1 | Pkt: PKT_9A4C01F2 | Hop: 1 | Dist: 142.3m | LoS: YES | SNR: 18.2 dB | Delay: 30ms
[SIM: 014.260s] [PKT_FORWARD ] Link: UAV_1 -> GCS   | Pkt: PKT_9A4C01F2 | Hop: 2 | Dist: 115.8m | LoS: YES | SNR: 21.6 dB | Delay: 30ms
[SIM: 014.260s] [PKT_DELIVERY] Dest: GCS   | Pkt: PKT_9A4C01F2 | Total Hops: 2 | Route: UAV_3 -> UAV_1 -> GCS | E2E Latency: 0.060s | Min SNR: 18.2 dB
```

When a link breaks and a packet is stored in buffer:
```text
[SIM: 018.450s] [LINK_BROKEN ] Link: UAV_3 -X- UAV_1 | Reason: Obstacle Occlusion by Building_04 | SNR: -12.4 dB (NLoS)
[SIM: 018.450s] [PKT_BUFFERED] Node: UAV_3 | Pkt: PKT_C81F003B | Reason: No active path to GCS | Buffer Occupancy: 12/250
[SIM: 021.100s] [ROUTE_FOUND ] Node: UAV_3 | New Path: UAV_3 -> UAV_2 -> GCS | Cost: 2.41 | Hops: 2
[SIM: 021.110s] [BUFFER_DRAIN] Node: UAV_3 | Flushed 12 packets along new route UAV_3 -> UAV_2 -> GCS
```

---

### 4.3 Network Performance Telemetry Metrics

The network subsystem computes real-time summary statistics exposed to HUD and test validation suites:

#### 1. Packet Delivery Ratio (PDR)
$$PDR = \frac{\sum \text{Packets Received at Destination}}{\sum \text{Packets Generated at Source Nodes}} \times 100\%$$
*Acceptance Target*: $\ge 95.0\%$ during nominal multi-hop surveying; $\ge 90.0\%$ during severe obstacle occlusion with cooperative relays.

#### 2. End-to-End Latency ($T_{e2e}$)
$$T_{e2e} = t_{delivered} - t_{created} = \sum_{h=1}^H \left(\tau_{prop} + \tau_{trans} + \tau_{queue} + \tau_{proc}\right)$$
- Transmission Delay ($\tau_{trans}$): $\frac{\text{Packet Size (bits)}}{\text{Channel Data Rate (bps)}} \approx \frac{1024 \times 8}{54 \times 10^6} \approx 0.15\text{ ms}$.
- Queuing & MAC Access Delay ($\tau_{queue}$): $10 - 25\text{ ms}$ per hop.
- Expected 2-Hop Latency: $30 - 60\text{ ms}$; Expected 3-Hop Latency: $60 - 100\text{ ms}$.

#### 3. Hop Count Distribution
Histogram categorizing all delivered packets:
- $H_1$: Direct 1-hop delivery ($UAV \to GCS$).
- $H_2$: 2-hop delivery ($UAV \to Relay \to GCS$).
- $H_3$: 3-hop delivery ($UAV \to Relay_1 \to Relay_2 \to GCS$).
- Multi-hop verification ratio: $\frac{H_2 + H_3 + \dots}{H_{total}} \times 100\%$. In distant surveying scenarios, this must exceed **$80\%$**.

#### 4. Path Stability & Churn Metric
$$\text{Churn Rate} = \frac{\text{Number of Route Flaps / Recalculations}}{\text{Mission Time (minutes)}}$$

---

### 4.4 Visual Link HUD & Color-Coding Specification (For Explorer 1 & UI)

To ensure intuitive visual presentation of the multi-hop network in the 3D graphics viewport, link lines between UAVs and GCS are color-coded based on instantaneous signal quality:

| Link State | SNR Range | Color Code | Line Style | Visual Meaning |
|---|---|---|---|---|
| **Strong Link** | $SNR \ge 18.0\text{ dB}$ | `#00FF66` (Vibrant Neon Green) | Solid, width 2.5px | Optimal throughput; near-zero packet loss. |
| **Moderate Link** | $10.0\text{ dB} \le SNR < 18.0\text{ dB}$ | `#FFCC00` (Amber Yellow) | Solid, width 2.0px | Adequate link; minor fading margin. |
| **Marginal / Fragile Link** | $5.0\text{ dB} \le SNR < 10.0\text{ dB}$ | `#FF6600` (Bright Orange) | Thin / Pulsing, width 1.5px | Near sensitivity cutoff; candidate for reroute. |
| **NLoS Occluded Link** | Blocked by Building | `#FF0033` (Red) or Invisible | Dashed / Low Alpha (0.2) | Blocked by disaster structure; non-transmitting. |
| **Active Routing Path** | Active Dijkstra Route | `#00E5FF` (Cyan Glow) | Highlighted thick line (3.5px) | The exact path currently carrying survey packets. |
| **In-Flight Packet Pulse** | Packet in transit | `#FFFFFF` (White Sphere / Pulse) | Animates along link | Represents packet moving hop-by-hop. |

---

## 5. Concrete Python Architecture & Class Interface Contracts

The following modular class contracts provide the exact blueprint for Milestone 2 implementation:

```python
"""
fanet_network.py - Architectural Interface Contracts
"""
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Any
import numpy as np

@dataclass
class ObstacleAABB:
    id: str
    min_pt: np.ndarray  # [x_min, y_min, z_min]
    max_pt: np.ndarray  # [x_max, y_max, z_max]

class ChannelModel:
    """Computes RF propagation, path loss, obstacle occlusion, and SNR."""
    def __init__(self, fc_ghz: float = 2.4, p_tx_dbm: float = 20.0, noise_floor_dbm: float = -96.0):
        self.fc_ghz = fc_ghz
        self.p_tx_dbm = p_tx_dbm
        self.noise_floor_dbm = noise_floor_dbm
        self.lambda_m = 0.3 / fc_ghz
        self.pl_d0 = 20.0 * np.log10(4.0 * np.pi / self.lambda_m)  # ~40 dB at 1m
        
    def check_los(self, p1: np.ndarray, p2: np.ndarray, obstacles: List[ObstacleAABB]) -> Tuple[bool, Optional[str]]:
        """Executes 3D ray-box slab intersection test."""
        ...
        
    def calculate_link(self, p1: np.ndarray, p2: np.ndarray, obstacles: List[ObstacleAABB]) -> Dict[str, Any]:
        """
        Returns dict containing:
        - 'distance_m': float
        - 'has_los': bool
        - 'path_loss_db': float
        - 'p_rx_dbm': float
        - 'snr_db': float
        - 'success_prob': float
        - 'is_connected': bool (snr >= 5.0 dB)
        """
        ...

class PacketBuffer:
    """Delay-Tolerant FIFO buffer with TTL expiration."""
    def __init__(self, max_capacity: int = 250, ttl_sec: float = 45.0):
        self.max_capacity = max_capacity
        self.ttl_sec = ttl_sec
        self.queue: List[NetworkPacket] = []
        
    def enqueue(self, pkt: NetworkPacket) -> bool:
        """Enqueues packet if space permits, pruning expired packets."""
        ...
        
    def dequeue_batch(self, count: int, current_time: float) -> List[NetworkPacket]:
        """Flushes up to `count` non-expired packets."""
        ...

class RoutingEngine:
    """Implements Dynamic Link-State Dijkstra (FANET-DLS)."""
    def __init__(self, node_id: str):
        self.node_id = node_id
        self.routing_table: Dict[str, RouteEntry] = {}
        
    def update_topology(self, adjacency_matrix: Dict[Tuple[str, str], float], current_time: float):
        """Re-computes Dijkstra shortest paths across all nodes."""
        ...
        
    def get_next_hop(self, dst_id: str) -> Optional[str]:
        """Returns next-hop node ID or None if unreachable."""
        ...

class NetworkManager:
    """Orchestrates all network nodes, channel propagation, packet delivery, and telemetry."""
    def __init__(self, node_positions: Dict[str, np.ndarray], obstacles: List[ObstacleAABB]):
        self.nodes = node_positions  # e.g. {"GCS": np.array([0,0,0]), "UAV_1": ...}
        self.obstacles = obstacles
        self.channel = ChannelModel()
        self.buffers: Dict[str, PacketBuffer] = {}
        self.telemetry = NetworkTelemetry()
        
    def step(self, dt: float, current_sim_time: float) -> Dict[str, Any]:
        """
        Simulation tick:
        1. Updates 3D node positions.
        2. Evaluates all pairwise links (distance, LoS, SNR).
        3. Updates routing topology graph.
        4. Transmits pending and buffered packets hop-by-hop.
        5. Logs multi-hop events and updates PDR/latency metrics.
        Returns visual snapshot for rendering engine.
        """
        ...
```

---

## 6. Verification & Test Plan for Multi-Hop Network

To ensure zero-regression, forensic verifiability, the multi-hop networking subsystem must pass a 4-tier validation suite:

1. **Unit Tests (RF & Channel Physics)**:
   - Verify Friis FSPL matches theoretical $40.05\text{ dB}$ at $1.0\text{ m}$.
   - Verify 3D Ray-Box intersection correctly flags obstruction when a building is placed directly between two UAVs.
   - Verify SNR and reception probability drop to zero under multi-building NLoS.
2. **Topology & Multi-Hop Dijkstra Tests**:
   - **Direct Link Test**: UAV within $100\text{ m}$ of GCS routes directly ($1\text{ hop}$).
   - **2-Hop Relay Test**: Survey UAV at $350\text{ m}$ (out of GCS range); Relay UAV placed at $175\text{ m}$. Verify route resolves strictly to `UAV_Survey -> UAV_Relay -> GCS`.
   - **Dynamic Failover Test**: Sever link between Relay 1 and GCS. Verify routing engine immediately failovers to Relay 2 within $< 100\text{ ms}$.
3. **DTN Buffer-and-Forward Tests**:
   - Temporarily place Survey UAV in RF shadow (all links severed). Generate 10 survey packets. Verify all 10 packets are buffered without loss.
   - Restore LoS link. Verify all 10 packets are drained in order and successfully delivered to GCS.
4. **End-to-End Mission Scenario Test**:
   - Fleet of 5 UAVs: 1 GCS, 2 Relay UAVs, 2 Survey UAVs surveying 4 distant PoIs.
   - Measure and assert:
     - Overall Packet Delivery Ratio $\ge 95\%$.
     - Total multi-hop packet share $\ge 80\%$.
     - Clear, uncorrupted hop logs in telemetry output.

---

## 7. Architectural Recommendations for Project Synthesis (PROJECT.md)

1. **Adopt Dynamic Link-State Dijkstra (FANET-DLS)**: Provides instantaneous packet forwarding, zero route discovery latency, and deterministic loop-free paths that are trivial to visualize in real time.
2. **Incorporate 3D Ray-AABB Obstacle Occlusion**: Essential for post-disaster realism. Direct rays passing through buildings must experience realistic building penetration loss ($+22\text{ dB}$) triggering multi-hop rerouting.
3. **Decouple Network Physics from Visualization**: The `NetworkManager` class should run purely on standard Python, `numpy`, and `networkx`, allowing full headless execution during automated CI/CD and unit testing while exposing a lightweight dict to the 3D renderer.
4. **Deploy Spring-Damper Cooperative Relays**: Relay UAVs should autonomously maintain intermediate spacing ($d \approx 180\text{ m}$) between GCS and active Survey UAVs with LoS elevation gradients to ensure persistent mission coverage.
