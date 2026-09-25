"""
UAV-X Mission Controller
========================
Async MAVSDK-Python controller for managing a heterogeneous fleet of UAVs
in the UAV-X disaster-response system.

Responsibilities:
  - Connecting to one or more PX4 SITL / real-UAV systems via MAVSDK
  - Arming, taking off, and switching flight modes
  - Uploading and starting waypoint missions for scout UAVs
  - Positioning relay UAVs at communication relay points
  - Continuous battery monitoring with configurable callback
  - Fetching consolidated telemetry snapshots
  - Emergency RTL and coordinated landing

Usage example
-------------
>>> import asyncio
>>> from mission_controller import MissionController
>>>
>>> async def main():
...     mc = MissionController()
...     addresses = {
...         "uav_0": "udp://:14540",
...         "uav_1": "udp://:14541",
...         "uav_2": "udp://:14542",
...     }
...     await mc.connect_all(addresses)
...     await mc.arm_and_takeoff("uav_0", altitude=30.0)
...     await mc.assign_scout("uav_0", poi_position=(47.398, 8.546, 30.0))
...     telem = await mc.get_telemetry("uav_0")
...     print(telem)
...
>>> asyncio.run(main())

Dependencies
------------
  pip install mavsdk   # requires PX4 SITL or real hardware
"""

import asyncio
import logging
from typing import Any, Callable, Coroutine, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Optional MAVSDK import – the module degrades gracefully if MAVSDK is absent
# so it can still be imported in unit-test / CI environments without hardware.
# ---------------------------------------------------------------------------
try:
    from mavsdk import System
    from mavsdk.action import ActionError
    from mavsdk.mission import (
        MissionItem,
        MissionPlan,
        MissionError,
    )
    from mavsdk.offboard import (
        OffboardError,
        PositionNedYaw,
        VelocityNedYaw,
    )
    from mavsdk.telemetry import FlightMode, LandedState
    _MAVSDK_AVAILABLE = True
except ImportError:
    _MAVSDK_AVAILABLE = False
    # Create lightweight stubs so the rest of the module can still be parsed
    # and used with the pure-Python simulation back-end.
    class System:          # type: ignore[no-redef]
        pass
    class ActionError(Exception): pass   # type: ignore[no-redef]
    class MissionError(Exception): pass  # type: ignore[no-redef]
    class OffboardError(Exception): pass # type: ignore[no-redef]
    class MissionItem:     # type: ignore[no-redef]
        pass
    class MissionPlan:     # type: ignore[no-redef]
        pass
    class FlightMode:      # type: ignore[no-redef]
        MISSION = "MISSION"
        HOLD    = "HOLD"
        OFFBOARD= "OFFBOARD"
    class LandedState:     # type: ignore[no-redef]
        ON_GROUND = "ON_GROUND"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("MissionController")


# ---------------------------------------------------------------------------
# Helper constants
# ---------------------------------------------------------------------------

#: Seconds to wait between health checks while connecting
_HEALTH_POLL_INTERVAL: float = 0.5

#: Default takeoff altitude above home position (metres)
_DEFAULT_TAKEOFF_ALT: float = 30.0

#: Loiter time at each mission waypoint (seconds)
_WAYPOINT_LOITER_SEC: float = 10.0

#: Acceptance radius for reaching a waypoint (metres)
_ACCEPTANCE_RADIUS_M: float = 3.0

#: Connection timeout in seconds
_CONNECT_TIMEOUT_SEC: float = 60.0


# ---------------------------------------------------------------------------
# MissionController
# ---------------------------------------------------------------------------

class MissionController:
    """
    High-level async controller for the UAV-X drone fleet.

    All public methods are coroutines and must be awaited inside an async
    context (``asyncio.run(...)`` or ``await`` from another coroutine).

    Attributes
    ----------
    drones : dict[str, System]
        Mapping from user-assigned UAV ID (e.g. ``"uav_0"``) to a connected
        MAVSDK :class:`System` object.

    Examples
    --------
    >>> mc = MissionController()
    >>> await mc.connect_all({"uav_0": "udp://:14540"})
    >>> await mc.arm_and_takeoff("uav_0", altitude=25.0)
    """

    def __init__(self) -> None:
        """Initialise an empty controller; call :meth:`connect_all` next."""
        self.drones: Dict[str, Any] = {}
        self._battery_monitors: Dict[str, asyncio.Task] = {}

        if not _MAVSDK_AVAILABLE:
            logger.warning(
                "MAVSDK not installed.  Install with:  pip install mavsdk\n"
                "Running in STUB mode – drone commands will be no-ops."
            )

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    async def connect_all(
        self,
        addresses: Dict[str, str],
        timeout: float = _CONNECT_TIMEOUT_SEC,
    ) -> None:
        """
        Connect to all UAVs concurrently and wait until each reports healthy.

        Parameters
        ----------
        addresses : dict[str, str]
            Mapping of UAV identifier → MAVSDK connection URL.
            Examples of valid URLs:
              * ``"udp://:14540"``   – listen on UDP port 14540
              * ``"tcp://192.168.1.10:5760"``
              * ``"serial:///dev/ttyUSB0:57600"``
        timeout : float
            Maximum seconds to wait for a UAV to become healthy.
            Raises :class:`TimeoutError` per UAV that does not respond.

        Raises
        ------
        TimeoutError
            If a UAV has not reported all-healthy within *timeout* seconds.
        RuntimeError
            If MAVSDK is not installed.

        Examples
        --------
        >>> await mc.connect_all({
        ...     "uav_0": "udp://:14540",
        ...     "uav_1": "udp://:14541",
        ... })
        """
        if not _MAVSDK_AVAILABLE:
            logger.error("connect_all() called but MAVSDK is not installed.")
            return

        async def _connect_one(uav_id: str, addr: str) -> None:
            logger.info("Connecting to %s at %s …", uav_id, addr)
            drone = System()
            await drone.connect(system_address=addr)

            # Poll health until all flags are True
            elapsed = 0.0
            async for health in drone.telemetry.health():
                if (
                    health.is_global_position_ok
                    and health.is_local_position_ok
                    and health.is_home_position_ok
                    and health.is_armable
                ):
                    break
                await asyncio.sleep(_HEALTH_POLL_INTERVAL)
                elapsed += _HEALTH_POLL_INTERVAL
                if elapsed >= timeout:
                    raise TimeoutError(
                        f"UAV '{uav_id}' did not become healthy within "
                        f"{timeout}s"
                    )

            self.drones[uav_id] = drone
            logger.info("✓ %s connected and healthy.", uav_id)

        # Run all connections concurrently
        tasks = [
            asyncio.create_task(_connect_one(uid, addr))
            for uid, addr in addresses.items()
        ]
        results = await asyncio.gather(*tasks, return_exceptions=True)

        for uav_id, result in zip(addresses.keys(), results):
            if isinstance(result, Exception):
                logger.error(
                    "Failed to connect to %s: %s", uav_id, result
                )

    # ------------------------------------------------------------------
    # Arm & takeoff
    # ------------------------------------------------------------------

    async def arm_and_takeoff(
        self,
        uav_id: str,
        altitude: float = _DEFAULT_TAKEOFF_ALT,
    ) -> None:
        """
        Arm the UAV and command it to take off to *altitude* metres AGL.

        The method waits until the drone reports it is in the air before
        returning, so callers can safely issue follow-on commands.

        Parameters
        ----------
        uav_id : str
            ID used when calling :meth:`connect_all`.
        altitude : float
            Target altitude in metres above ground level.

        Raises
        ------
        KeyError
            If *uav_id* was not registered via :meth:`connect_all`.
        ActionError
            If the PX4 firmware rejects the arm or takeoff command.

        Examples
        --------
        >>> await mc.arm_and_takeoff("uav_0", altitude=40.0)
        """
        drone = self._get_drone(uav_id)

        try:
            logger.info("[%s] Arming …", uav_id)
            await drone.action.arm()

            logger.info("[%s] Taking off to %.1f m …", uav_id, altitude)
            await drone.action.set_takeoff_altitude(altitude)
            await drone.action.takeoff()

            # Wait until airborne
            async for state in drone.telemetry.landed_state():
                if state == LandedState.IN_AIR:
                    break

            logger.info("[%s] Airborne at %.1f m.", uav_id, altitude)

        except ActionError as exc:
            logger.error("[%s] arm_and_takeoff failed: %s", uav_id, exc)
            raise

    # ------------------------------------------------------------------
    # Scout mission
    # ------------------------------------------------------------------

    async def assign_scout(
        self,
        uav_id: str,
        poi_position: Tuple[float, float, float],
    ) -> None:
        """
        Upload and start a single-waypoint scout mission to a Point-of-Interest.

        The UAV will fly to *poi_position*, loiter for
        :data:`_WAYPOINT_LOITER_SEC` seconds, then hold position awaiting
        further instructions.

        Parameters
        ----------
        uav_id : str
            Target UAV identifier.
        poi_position : tuple[float, float, float]
            ``(latitude_deg, longitude_deg, altitude_m)`` of the PoI.

        Raises
        ------
        MissionError
            If uploading or starting the mission fails.

        Examples
        --------
        >>> await mc.assign_scout("uav_0", poi_position=(47.3987, 8.5462, 30.0))
        """
        drone = self._get_drone(uav_id)
        lat, lon, alt = poi_position

        logger.info(
            "[%s] Building scout mission → PoI (%.6f, %.6f, %.1f m)",
            uav_id, lat, lon, alt,
        )

        # Build a single MissionItem at the PoI coordinates
        mission_item = MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=alt,
            speed_m_s=10.0,
            is_fly_through=False,
            gimbal_pitch_deg=-90.0,   # Camera pointing straight down
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.TAKE_PHOTO,
            loiter_time_s=_WAYPOINT_LOITER_SEC,
            camera_photo_interval_s=2.0,
            acceptance_radius_m=_ACCEPTANCE_RADIUS_M,
            yaw_deg=float("nan"),
            camera_photo_distance_m=float("nan"),
            vehicle_action=MissionItem.VehicleAction.NONE,
        )

        mission_plan = MissionPlan(mission_items=[mission_item])

        try:
            logger.info("[%s] Uploading mission …", uav_id)
            await drone.mission.upload_mission(mission_plan)
            await drone.action.arm()          # Re-arm if needed (no-op if already)
            await drone.mission.start_mission()
            logger.info("[%s] Scout mission started.", uav_id)
        except MissionError as exc:
            logger.error("[%s] assign_scout failed: %s", uav_id, exc)
            raise

    # ------------------------------------------------------------------
    # Relay positioning
    # ------------------------------------------------------------------

    async def assign_relay(
        self,
        uav_id: str,
        relay_position: Tuple[float, float, float],
    ) -> None:
        """
        Fly a UAV to a communication relay position and hold there.

        Internally this uploads a single-waypoint mission (``is_fly_through=True``
        so the drone does not loiter) and then switches the vehicle to HOLD
        mode once it reaches the waypoint.

        Parameters
        ----------
        uav_id : str
            Target UAV identifier.
        relay_position : tuple[float, float, float]
            ``(latitude_deg, longitude_deg, altitude_m)`` of the relay point.

        Raises
        ------
        MissionError
            If the mission upload fails.
        ActionError
            If mode switch fails.

        Examples
        --------
        >>> await mc.assign_relay("uav_1", relay_position=(47.3990, 8.5470, 50.0))
        """
        drone = self._get_drone(uav_id)
        lat, lon, alt = relay_position

        logger.info(
            "[%s] Assigning relay position (%.6f, %.6f, %.1f m)",
            uav_id, lat, lon, alt,
        )

        relay_item = MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=alt,
            speed_m_s=12.0,
            is_fly_through=True,           # Don't loiter – just pass through
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0,
            camera_photo_interval_s=0.0,
            acceptance_radius_m=_ACCEPTANCE_RADIUS_M,
            yaw_deg=float("nan"),
            camera_photo_distance_m=float("nan"),
            vehicle_action=MissionItem.VehicleAction.NONE,
        )

        mission_plan = MissionPlan(mission_items=[relay_item])

        try:
            await drone.mission.upload_mission(mission_plan)
            await drone.mission.start_mission()

            # Wait until mission is finished, then switch to HOLD
            async for progress in drone.mission.mission_progress():
                if progress.current == progress.total:
                    break

            await drone.action.hold()
            logger.info("[%s] Relay position reached, holding.", uav_id)

        except (MissionError, ActionError) as exc:
            logger.error("[%s] assign_relay failed: %s", uav_id, exc)
            raise

    # ------------------------------------------------------------------
    # Battery monitoring
    # ------------------------------------------------------------------

    async def monitor_battery(
        self,
        uav_id: str,
        threshold_pct: float,
        callback: Callable[[str, float], Coroutine],
    ) -> None:
        """
        Continuously monitor a UAV's battery and invoke *callback* when low.

        This coroutine runs indefinitely (until cancelled) as an async
        generator over :attr:`telemetry.battery`.  Call it with
        ``asyncio.create_task(...)`` to run it concurrently.

        Parameters
        ----------
        uav_id : str
            Target UAV identifier.
        threshold_pct : float
            Battery percentage (0–100) below which *callback* is invoked.
            The callback is called once every time a sample drops below
            the threshold (it is the caller's responsibility to rate-limit
            if needed).
        callback : async callable (uav_id: str, remaining_pct: float) -> None
            Async function invoked when the battery is critically low.
            Typical actions: trigger RTL, notify GCS, log alert.

        Examples
        --------
        >>> async def on_low_battery(uid, pct):
        ...     print(f"{uid} battery critical: {pct:.1f}%")
        ...     await mc.emergency_rtl(uid)
        ...
        >>> task = asyncio.create_task(
        ...     mc.monitor_battery("uav_0", threshold_pct=20.0, callback=on_low_battery)
        ... )
        """
        drone = self._get_drone(uav_id)
        logger.info(
            "[%s] Battery monitor started (threshold=%.1f%%)",
            uav_id, threshold_pct,
        )

        try:
            async for battery in drone.telemetry.battery():
                remaining = (battery.remaining_percent or 0.0) * 100.0
                if remaining < threshold_pct:
                    logger.warning(
                        "[%s] LOW BATTERY: %.1f%% < threshold %.1f%%",
                        uav_id, remaining, threshold_pct,
                    )
                    # Fire the callback asynchronously
                    await callback(uav_id, remaining)
        except asyncio.CancelledError:
            logger.info("[%s] Battery monitor cancelled.", uav_id)
            raise

    # ------------------------------------------------------------------
    # Telemetry snapshot
    # ------------------------------------------------------------------

    async def get_telemetry(self, uav_id: str) -> Dict[str, Any]:
        """
        Return a consolidated telemetry snapshot for *uav_id*.

        Fetches one sample from each relevant telemetry stream concurrently
        and returns a dictionary.

        Parameters
        ----------
        uav_id : str
            Target UAV identifier.

        Returns
        -------
        dict with keys:
          * ``position``    – ``{"lat": float, "lon": float, "abs_alt": float, "rel_alt": float}``
          * ``battery_pct`` – remaining battery as 0-100 float
          * ``armed``       – ``bool``
          * ``flight_mode`` – :class:`mavsdk.telemetry.FlightMode` enum value (string repr)
          * ``is_in_air``   – ``bool``
          * ``heading_deg`` – compass heading in degrees

        Examples
        --------
        >>> telem = await mc.get_telemetry("uav_0")
        >>> print(f"Battery: {telem['battery_pct']:.1f}%")
        """
        drone = self._get_drone(uav_id)

        async def _first(stream):
            """Pull exactly one value from an async generator."""
            async for val in stream:
                return val

        # Fetch all streams concurrently
        results = await asyncio.gather(
            _first(drone.telemetry.position()),
            _first(drone.telemetry.battery()),
            _first(drone.telemetry.armed()),
            _first(drone.telemetry.flight_mode()),
            _first(drone.telemetry.in_air()),
            _first(drone.telemetry.heading()),
            return_exceptions=True,
        )

        position, battery, armed, flight_mode, in_air, heading = results

        def _safe(val, default=None):
            return default if isinstance(val, Exception) else val

        pos = _safe(position)
        bat = _safe(battery)
        arm = _safe(armed, False)
        fm  = _safe(flight_mode, "UNKNOWN")
        air = _safe(in_air, False)
        hdg = _safe(heading)

        return {
            "position": {
                "lat":     pos.latitude_deg       if pos else None,
                "lon":     pos.longitude_deg      if pos else None,
                "abs_alt": pos.absolute_altitude_m if pos else None,
                "rel_alt": pos.relative_altitude_m if pos else None,
            },
            "battery_pct": (bat.remaining_percent * 100.0) if bat else None,
            "armed":       bool(arm),
            "flight_mode": str(fm),
            "is_in_air":   bool(air),
            "heading_deg": hdg.heading_deg if hdg else None,
        }

    # ------------------------------------------------------------------
    # Emergency RTL
    # ------------------------------------------------------------------

    async def emergency_rtl(self, uav_id: str) -> None:
        """
        Command an immediate Return-to-Launch for *uav_id*.

        Sends :meth:`action.return_to_launch()` and logs the event.
        This is a fire-and-forget command; the UAV firmware handles the
        RTL trajectory autonomously.

        Parameters
        ----------
        uav_id : str
            Target UAV identifier.

        Examples
        --------
        >>> await mc.emergency_rtl("uav_0")
        """
        drone = self._get_drone(uav_id)
        try:
            logger.warning("[%s] ⚠ EMERGENCY RTL commanded!", uav_id)
            await drone.action.return_to_launch()
        except ActionError as exc:
            logger.error("[%s] RTL failed: %s", uav_id, exc)
            raise

    # ------------------------------------------------------------------
    # Land all
    # ------------------------------------------------------------------

    async def land_all(self) -> None:
        """
        Command all connected UAVs to land simultaneously.

        Iterates over :attr:`drones` and sends :meth:`action.land()` to
        each.  Errors for individual drones are logged but do not prevent
        the others from receiving the command.

        Examples
        --------
        >>> await mc.land_all()
        """
        if not self.drones:
            logger.warning("land_all() called but no drones are connected.")
            return

        async def _land_one(uid: str, drone: Any) -> None:
            try:
                logger.info("[%s] Landing …", uid)
                await drone.action.land()
            except ActionError as exc:
                logger.error("[%s] land failed: %s", uid, exc)

        await asyncio.gather(
            *[_land_one(uid, drone) for uid, drone in self.drones.items()]
        )
        logger.info("Land commands sent to all %d UAVs.", len(self.drones))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_drone(self, uav_id: str) -> Any:
        """
        Retrieve a connected drone by ID, raising :class:`KeyError` if not
        found.

        Parameters
        ----------
        uav_id : str
            UAV identifier registered via :meth:`connect_all`.

        Returns
        -------
        mavsdk.System
        """
        if uav_id not in self.drones:
            raise KeyError(
                f"UAV '{uav_id}' is not connected.  "
                f"Known UAVs: {list(self.drones.keys())}"
            )
        return self.drones[uav_id]

    def list_connected(self) -> List[str]:
        """Return a list of all currently connected UAV IDs."""
        return list(self.drones.keys())

    async def cancel_battery_monitor(self, uav_id: str) -> None:
        """Cancel a running battery-monitor task for *uav_id*, if any."""
        task = self._battery_monitors.pop(uav_id, None)
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            logger.info("[%s] Battery monitor cancelled.", uav_id)


# ---------------------------------------------------------------------------
# Demo entry-point
# ---------------------------------------------------------------------------

async def _demo() -> None:
    """
    Demonstration: connect to 3 simulated PX4 SITL UAVs, take off, assign
    roles, monitor telemetry, then land.

    Start 3 SITL instances first:
        $ make px4_sitl_default gazebo HEADLESS=1 &   # port 14540
        $ PX4_SIM_PORT=14541 make px4_sitl_default gazebo HEADLESS=1 &
        $ PX4_SIM_PORT=14542 make px4_sitl_default gazebo HEADLESS=1 &
    """
    if not _MAVSDK_AVAILABLE:
        logger.error(
            "MAVSDK is not installed.  Run:  pip install mavsdk\n"
            "Cannot run demo without MAVSDK."
        )
        return

    mc = MissionController()

    # ------------------------------------------------------------------ #
    # 1. Connect to three simulated UAVs                                   #
    # ------------------------------------------------------------------ #
    addresses = {
        "uav_0": "udp://:14540",
        "uav_1": "udp://:14541",
        "uav_2": "udp://:14542",
    }
    logger.info("=== UAV-X Mission Controller Demo ===")
    await mc.connect_all(addresses, timeout=60.0)

    connected = mc.list_connected()
    logger.info("Connected UAVs: %s", connected)
    if not connected:
        logger.error("No UAVs connected – aborting demo.")
        return

    # ------------------------------------------------------------------ #
    # 2. Arm and take off                                                  #
    # ------------------------------------------------------------------ #
    takeoff_tasks = [mc.arm_and_takeoff(uid, altitude=30.0) for uid in connected]
    await asyncio.gather(*takeoff_tasks)

    # ------------------------------------------------------------------ #
    # 3. Assign roles                                                      #
    # ------------------------------------------------------------------ #
    # uav_0 → scout a PoI
    if "uav_0" in mc.drones:
        await mc.assign_scout(
            "uav_0",
            poi_position=(47.3987, 8.5462, 30.0),
        )

    # uav_1 → communication relay
    if "uav_1" in mc.drones:
        await mc.assign_relay(
            "uav_1",
            relay_position=(47.3985, 8.5480, 50.0),
        )

    # ------------------------------------------------------------------ #
    # 4. Battery monitoring (async tasks)                                  #
    # ------------------------------------------------------------------ #
    async def on_low_battery(uid: str, pct: float) -> None:
        logger.warning("!!! %s battery at %.1f%% — triggering RTL !!!", uid, pct)
        await mc.emergency_rtl(uid)

    monitor_tasks = []
    for uid in connected:
        task = asyncio.create_task(
            mc.monitor_battery(uid, threshold_pct=20.0, callback=on_low_battery)
        )
        mc._battery_monitors[uid] = task
        monitor_tasks.append(task)

    # ------------------------------------------------------------------ #
    # 5. Telemetry snapshot loop (30 seconds)                              #
    # ------------------------------------------------------------------ #
    logger.info("Polling telemetry for 30 s …")
    for _ in range(6):
        await asyncio.sleep(5)
        for uid in connected:
            try:
                telem = await mc.get_telemetry(uid)
                pos = telem["position"]
                logger.info(
                    "[%s] lat=%.5f lon=%.5f alt=%.1f batt=%.1f%% mode=%s",
                    uid,
                    pos["lat"] or 0.0,
                    pos["lon"] or 0.0,
                    pos["rel_alt"] or 0.0,
                    telem["battery_pct"] or 0.0,
                    telem["flight_mode"],
                )
            except Exception as exc:
                logger.warning("[%s] telemetry error: %s", uid, exc)

    # ------------------------------------------------------------------ #
    # 6. Land all and clean up                                             #
    # ------------------------------------------------------------------ #
    logger.info("Commanding all UAVs to land …")
    for task in monitor_tasks:
        task.cancel()
    await mc.land_all()
    logger.info("=== Demo complete ===")


if __name__ == "__main__":
    asyncio.run(_demo())
