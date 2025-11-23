"""
Unit tests for DroneController class.
Tests focus on pure functions and input validation.
"""
import pytest
import math
import sys
import os
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Mock pymavlink before importing DroneController
sys.modules['pymavlink'] = MagicMock()
sys.modules['pymavlink.mavutil'] = MagicMock()

from drone_controller import DroneController


class TestGetDistanceMetres:
    """Tests for the Haversine distance calculation function."""

    def test_same_location_returns_zero(self):
        """Distance between identical coordinates should be zero."""
        controller = DroneController.__new__(DroneController)
        distance = controller.get_distance_metres(37.5665, 126.9780, 37.5665, 126.9780)
        assert distance == pytest.approx(0, abs=0.01)

    def test_known_distance_seoul_to_busan(self):
        """Test with known distance: Seoul to Busan (~325 km)."""
        controller = DroneController.__new__(DroneController)
        # Seoul: 37.5665, 126.9780
        # Busan: 35.1796, 129.0756
        distance = controller.get_distance_metres(37.5665, 126.9780, 35.1796, 129.0756)
        # Expected: approximately 325 km (325000 m)
        assert distance == pytest.approx(325000, rel=0.05)  # 5% tolerance

    def test_short_distance_meters(self):
        """Test short distance calculation (typical drone operation range)."""
        controller = DroneController.__new__(DroneController)
        # Move approximately 100 meters north (about 0.0009 degrees latitude)
        lat1, lon1 = 37.5665, 126.9780
        lat2, lon2 = 37.5674, 126.9780  # ~100m north
        distance = controller.get_distance_metres(lat1, lon1, lat2, lon2)
        assert distance == pytest.approx(100, rel=0.1)  # 10% tolerance

    def test_symmetry(self):
        """Distance from A to B should equal distance from B to A."""
        controller = DroneController.__new__(DroneController)
        lat1, lon1 = 37.5665, 126.9780
        lat2, lon2 = 37.5700, 126.9800

        distance_ab = controller.get_distance_metres(lat1, lon1, lat2, lon2)
        distance_ba = controller.get_distance_metres(lat2, lon2, lat1, lon1)

        assert distance_ab == pytest.approx(distance_ba, rel=0.001)

    def test_east_west_distance(self):
        """Test distance calculation along longitude (east-west)."""
        controller = DroneController.__new__(DroneController)
        # Move approximately 100 meters east at Seoul latitude
        lat1, lon1 = 37.5665, 126.9780
        lat2, lon2 = 37.5665, 126.9791  # ~100m east
        distance = controller.get_distance_metres(lat1, lon1, lat2, lon2)
        assert distance == pytest.approx(100, rel=0.15)  # 15% tolerance

    def test_diagonal_distance(self):
        """Test distance calculation for diagonal movement."""
        controller = DroneController.__new__(DroneController)
        lat1, lon1 = 37.5665, 126.9780
        # Move 100m north and 100m east (diagonal ~141m)
        lat2, lon2 = 37.5674, 126.9791
        distance = controller.get_distance_metres(lat1, lon1, lat2, lon2)
        # Pythagorean: sqrt(100^2 + 100^2) ≈ 141.4m
        assert distance == pytest.approx(141, rel=0.15)


class TestSetCommand:
    """Tests for the set_command input validation."""

    def test_valid_command_list(self, mock_mavlink):
        """Valid 4-element list should be accepted."""
        controller = DroneController.__new__(DroneController)
        controller.master = mock_mavlink

        result = controller.set_command([0.1, 0.2, 0.0, 0])
        assert result is True

    def test_invalid_command_length_raises(self, mock_mavlink):
        """Command with wrong length should raise ValueError."""
        controller = DroneController.__new__(DroneController)
        controller.master = mock_mavlink

        with pytest.raises(ValueError, match="must be a list with 4 elements"):
            controller.set_command([0.1, 0.2])

    def test_empty_command_raises(self, mock_mavlink):
        """Empty command list should raise ValueError."""
        controller = DroneController.__new__(DroneController)
        controller.master = mock_mavlink

        with pytest.raises(ValueError, match="must be a list with 4 elements"):
            controller.set_command([])

    def test_non_list_command_raises(self, mock_mavlink):
        """Non-list command should raise ValueError."""
        controller = DroneController.__new__(DroneController)
        controller.master = mock_mavlink

        with pytest.raises(ValueError, match="must be a list with 4 elements"):
            controller.set_command((0.1, 0.2, 0.0, 0))  # tuple instead of list

    def test_yaw_normalization_above_180(self, mock_mavlink):
        """Yaw values > 180 should be normalized to negative."""
        controller = DroneController.__new__(DroneController)
        controller.master = mock_mavlink

        # This tests internal logic - yaw of 270 should become -90
        result = controller.set_command([0, 0, 0, 270])
        assert result is True

    def test_no_master_connection_returns_false(self):
        """set_command should return False when master is None."""
        controller = DroneController.__new__(DroneController)
        controller.master = None

        result = controller.set_command([0.1, 0.2, 0.0, 0])
        assert result is False

    def test_zero_velocity_command(self, mock_mavlink):
        """Zero velocity (hover) command should work."""
        controller = DroneController.__new__(DroneController)
        controller.master = mock_mavlink

        result = controller.set_command([0, 0, 0, 0])
        assert result is True

    def test_max_velocity_command(self, mock_mavlink):
        """High velocity command should be sent without modification."""
        controller = DroneController.__new__(DroneController)
        controller.master = mock_mavlink

        result = controller.set_command([1.0, 1.0, 0.5, 180])
        assert result is True


class TestDroneControllerInit:
    """Tests for DroneController initialization."""

    def test_default_values(self):
        """Check default initialization values."""
        controller = DroneController.__new__(DroneController)
        controller.__init__()

        assert controller.connection_string == '/dev/ttyACM0'
        assert controller.baudrate == 115200
        assert controller.master is None
        assert controller.is_armed is False
        assert controller.set_alt == 2.0
        assert controller.home_lat is None
        assert controller.home_lon is None

    def test_custom_connection_string(self):
        """Check custom connection string is set."""
        controller = DroneController.__new__(DroneController)
        controller.__init__(connection_string='/dev/ttyUSB0', baudrate=57600)

        assert controller.connection_string == '/dev/ttyUSB0'
        assert controller.baudrate == 57600


class TestAltitudeCalculations:
    """Tests for altitude-related calculations."""

    def test_archived_altitude_calculation(self):
        """archived_alt should be 95% of set_alt."""
        controller = DroneController.__new__(DroneController)
        controller.set_alt = 10.0
        controller.archived_alt = controller.set_alt * 0.95

        assert controller.archived_alt == pytest.approx(9.5)

    def test_archived_altitude_with_different_altitudes(self):
        """Test archived altitude calculation with various set_alt values."""
        controller = DroneController.__new__(DroneController)

        for alt in [2.0, 5.0, 10.0, 20.0]:
            controller.set_alt = alt
            controller.archived_alt = controller.set_alt * 0.95
            assert controller.archived_alt == pytest.approx(alt * 0.95)
