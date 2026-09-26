"""
tests/unit/test_dual_band_mac.py: Unit tests for Dual-Band 2.4 GHz + 915 MHz LoRa & ETX Routing.
"""

import numpy as np
import pytest

from sim.network import DualBandRFChannelModel, FANETNetworkEngine
from sim.obstacles import ObstacleAABB


def test_dual_band_penetration_and_range():
    """Verify 915 MHz Sub-GHz band achieves lower penetration loss and longer reach than 2.4 GHz."""
    dual = DualBandRFChannelModel()

    # Case 1: Clear LoS at 100m -> Both viable, 2.4 GHz preferred for bandwidth
    eval_los = dual.evaluate_link(distance=100.0, is_los=True, num_occlusions=0)
    assert eval_los["viable"] is True
    assert eval_los["band"] == "2.4GHz"
    assert eval_los["throughput_mbps"] == 54.0
    assert eval_los["etx"] == pytest.approx(1.0, abs=0.2)

    # Case 2: Deep building occlusion (2 occlusions) at 150m
    # 2.4 GHz has 2 * 22dB = 44dB loss -> SNR drops below threshold
    # 915 MHz has 2 * 8dB = 16dB loss -> Remains viable as C2 fallback!
    eval_nlos = dual.evaluate_link(distance=150.0, is_los=False, num_occlusions=2)
    assert eval_nlos["viable"] is True
    assert eval_nlos["band"] == "915MHz"
    assert eval_nlos["throughput_mbps"] == 0.25  # Sub-GHz C2 fallback active!


def test_etx_link_quality_metric():
    """Verify ETX scales inversely with SNR (high SNR -> ETX ~ 1.0; low SNR -> high ETX)."""
    dual = DualBandRFChannelModel()

    etx_high_snr = dual.compute_etx(snr=25.0)
    etx_med_snr = dual.compute_etx(snr=6.0)
    etx_low_snr = dual.compute_etx(snr=1.0)

    assert etx_high_snr == pytest.approx(1.0, abs=0.05)
    assert etx_med_snr > etx_high_snr
    assert etx_low_snr > etx_med_snr


def test_dual_band_network_engine_integration():
    """Verify FANETNetworkEngine reports band and ETX in its active link table and metrics."""
    engine = FANETNetworkEngine()
    building = ObstacleAABB("BLD", "Dense Concrete", np.array([20.0, -10.0, 0.0]), np.array([60.0, 10.0, 40.0]))

    positions = {
        "GCS": np.array([0.0, 0.0, 0.0]),
        "UAV_1": np.array([100.0, 0.0, 20.0]),  # Occluded by building
    }

    engine.update_topology(positions, [building])
    link_info = engine.link_cache.get(("GCS", "UAV_1"))

    assert link_info is not None
    assert "band" in link_info
    assert "etx" in link_info
    assert link_info["etx"] >= 1.0

    metrics = engine.get_metrics()
    assert "band_24ghz_links" in metrics
    assert "band_915mhz_links" in metrics
    assert "mean_etx" in metrics
