"""
Proposed test additions for tests/unit/test_sim_core.py
Appended to end of tests/unit/test_sim_core.py to test aerodynamic downwash cone dynamics.
"""

def test_downwash_vertical_separation_concentric():
    """
    Verify downwash cone behavior in SwarmSimulationCore when lower drone is
    directly beneath upper drone (dz = -4.0m, d_xy = 0.0m).
    
    Validates:
    1. Execution does not throw NameError (verifies import math).
    2. Lower drone receives non-zero deterministic lateral escape force (> 10.0 N) along +X.
    3. Lower drone receives downward turbulence sink force (fz < 0.0 N).
    4. Upper drone experiences 0.0 downwash force (asymmetric aerodynamic wash).
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    # Compute steering forces
    f_bot = core.compute_steering_forces(d_bot)
    f_top = core.compute_steering_forces(d_top)

    # Lower drone: lateral escape along +X must be non-zero and substantial
    assert f_bot[0] > 10.0, f"Lower drone did not receive sufficient lateral escape force: {f_bot}"
    # Lower drone: downward sink force must be negative
    assert f_bot[2] < 0.0, f"Lower drone did not receive downward downwash sink: {f_bot}"

    # Upper drone: must feel zero downwash (only separation force upwards if within 6m)
    # At 4m distance, separation force on upper drone is along +Z (pos_top - pos_bot = +4m)
    # Downwash component on upper drone must be 0
    assert f_top[0] == pytest.approx(0.0, abs=1e-5)
    assert f_top[1] == pytest.approx(0.0, abs=1e-5)


def test_downwash_with_lateral_offset_in_core():
    """
    Verify downwash radial push on lower drone when laterally offset inside cone
    (dz = -4.0m, dx = 1.0m, cone radius = 4 * 0.4663 + 1.0 = 2.865m).
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([1.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    # Lower drone must be pushed outward along +X away from (0, 0)
    assert f_bot[0] > 0.0
    assert f_bot[2] < 0.0


def test_downwash_outside_cone_in_core():
    """
    Verify that when lower drone is outside the 25-degree downwash cone
    (dz = -4.0m, dx = 5.0m > r_cone = 2.865m), downwash force is 0.
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    # Place drone at 7m distance (outside 6m separation zone and outside downwash cone)
    d_bot = Drone("UAV_BOT", initial_position=np.array([7.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    assert np.allclose(f_bot, 0.0)


def test_downwash_disabled_configuration():
    """Verify that when enable_downwash=False, no downwash force is computed."""
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=False))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    # Only vertical separation force exists (along -Z), zero lateral force
    assert f_bot[0] == pytest.approx(0.0, abs=1e-6)
    assert f_bot[1] == pytest.approx(0.0, abs=1e-6)


def test_downwash_dynamic_lateral_escape_step():
    """
    Verify full dynamic simulation step: lower drone escapes the concentric
    downwash column across successive simulation steps.
    """
    core = SwarmSimulationCore(SimulationConfig(dt=0.05, enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    initial_x = d_bot.position[0]
    for _ in range(20):  # 1.0 second simulation
        core.step(0.05)

    # Lower drone must have moved significantly along +X (> 1.0 meter)
    assert d_bot.position[0] - initial_x > 1.0, f"Lower drone failed to dynamically escape downwash cone: {d_bot.position}"
