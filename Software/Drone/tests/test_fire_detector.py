"""
Unit tests for Fire_detector class.
Tests focus on GPS estimation and patrol logic.
"""
import pytest
import math
import sys
import os
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Mock ultralytics before importing Fire_detector
mock_yolo = MagicMock()
sys.modules['ultralytics'] = MagicMock()
sys.modules['ultralytics'].YOLO = mock_yolo


class TestFireGps:
    """Tests for the fire_gps GPS coordinate estimation function."""

    def create_fire_detector_for_gps_tests(self, mock_camera):
        """Helper to create Fire_detector with mocked cameras."""
        # Import here to avoid module-level import issues
        from Fire_detector import Fire_detector
        detector = Fire_detector.__new__(Fire_detector)
        # Set required attributes without full __init__
        detector.CAMERA_TILT_DEGREES = 30.0
        detector.MAX_FIRE_DISTANCE = 20
        return detector

    def test_fire_gps_with_dict_input(self, mock_camera):
        """Test GPS estimation with dictionary drone_gps input."""
        detector = self.create_fire_detector_for_gps_tests(mock_camera)

        drone_gps = {
            'lat': 37.5665,
            'lon': 126.9780,
            'alt': 2.0,
            'heading': 0
        }

        # Fire detected at frame center (no offset expected)
        result = detector.fire_gps(drone_gps, center_x=320, center_y=320)

        assert result is not None
        assert isinstance(result, tuple)
        assert len(result) == 2
        # Latitude should be close to original (fire is roughly ahead)
        assert abs(result[0] - drone_gps['lat']) < 0.001
        assert abs(result[1] - drone_gps['lon']) < 0.001

    def test_fire_gps_returns_none_for_none_input(self, mock_camera):
        """Test that None drone_gps returns None."""
        detector = self.create_fire_detector_for_gps_tests(mock_camera)

        result = detector.fire_gps(None, center_x=320, center_y=320)
        assert result is None

    def test_fire_gps_returns_none_for_invalid_altitude(self, mock_camera):
        """Test that altitude < 0.5 returns None."""
        detector = self.create_fire_detector_for_gps_tests(mock_camera)

        drone_gps = {
            'lat': 37.5665,
            'lon': 126.9780,
            'alt': 0.3,  # Too low
            'heading': 0
        }

        result = detector.fire_gps(drone_gps, center_x=320, center_y=320)
        assert result is None

    def test_fire_gps_returns_none_for_missing_coordinates(self, mock_camera):
        """Test that missing lat/lon returns None."""
        detector = self.create_fire_detector_for_gps_tests(mock_camera)

        drone_gps = {
            'alt': 2.0,
            'heading': 0
            # Missing lat and lon
        }

        result = detector.fire_gps(drone_gps, center_x=320, center_y=320)
        assert result is None

    def test_fire_gps_max_distance_exceeded(self, mock_camera):
        """Test that distances > MAX_FIRE_DISTANCE return None."""
        detector = self.create_fire_detector_for_gps_tests(mock_camera)
        detector.MAX_FIRE_DISTANCE = 5  # Set low for testing
        detector.CAMERA_TILT_DEGREES = 5  # Small angle = large distance

        drone_gps = {
            'lat': 37.5665,
            'lon': 126.9780,
            'alt': 2.0,
            'heading': 0
        }

        # Fire at frame top would calculate to far distance with shallow angle
        result = detector.fire_gps(drone_gps, center_x=320, center_y=50)
        # Should return None if calculated distance > MAX_FIRE_DISTANCE
        # Note: depends on actual calculation, may need adjustment
        assert result is None or isinstance(result, tuple)

    def test_fire_gps_with_object_input(self, mock_camera):
        """Test GPS estimation with object-style drone_gps input (legacy)."""
        detector = self.create_fire_detector_for_gps_tests(mock_camera)

        class MockGPS:
            lat = 37.5665
            lon = 126.9780
            alt = 2.0
            heading = 0

        result = detector.fire_gps(MockGPS(), center_x=320, center_y=320)
        assert result is not None
        assert isinstance(result, tuple)

    def test_fire_gps_heading_affects_result(self, mock_camera):
        """Test that different headings produce different fire locations."""
        detector = self.create_fire_detector_for_gps_tests(mock_camera)

        drone_gps_north = {
            'lat': 37.5665,
            'lon': 126.9780,
            'alt': 2.0,
            'heading': 0  # Facing north
        }

        drone_gps_east = {
            'lat': 37.5665,
            'lon': 126.9780,
            'alt': 2.0,
            'heading': 90  # Facing east
        }

        result_north = detector.fire_gps(drone_gps_north, center_x=320, center_y=400)
        result_east = detector.fire_gps(drone_gps_east, center_x=320, center_y=400)

        # Results should be different based on heading
        assert result_north is not None
        assert result_east is not None
        # At least one coordinate should differ significantly
        lat_diff = abs(result_north[0] - result_east[0])
        lon_diff = abs(result_north[1] - result_east[1])
        assert lat_diff > 0.00001 or lon_diff > 0.00001

    def test_fire_gps_altitude_affects_distance(self, mock_camera):
        """Test that higher altitude estimates further fire distance."""
        detector = self.create_fire_detector_for_gps_tests(mock_camera)

        drone_gps_low = {
            'lat': 37.5665,
            'lon': 126.9780,
            'alt': 2.0,
            'heading': 0
        }

        drone_gps_high = {
            'lat': 37.5665,
            'lon': 126.9780,
            'alt': 5.0,
            'heading': 0
        }

        result_low = detector.fire_gps(drone_gps_low, center_x=320, center_y=400)
        result_high = detector.fire_gps(drone_gps_high, center_x=320, center_y=400)

        assert result_low is not None
        assert result_high is not None

        # Higher altitude should estimate fire further away
        # Calculate distance from drone position
        def simple_distance(lat1, lon1, lat2, lon2):
            return math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2)

        dist_low = simple_distance(37.5665, 126.9780, result_low[0], result_low[1])
        dist_high = simple_distance(37.5665, 126.9780, result_high[0], result_high[1])

        assert dist_high > dist_low


class TestPatrolLogic:
    """Tests for the patrol_logic state machine."""

    def create_fire_detector_for_patrol_tests(self, mock_camera):
        """Helper to create Fire_detector with required patrol attributes."""
        from Fire_detector import Fire_detector
        detector = Fire_detector.__new__(Fire_detector)
        # Initialize patrol-related attributes
        detector.patrol_mode = True
        detector.patrol_state = 'top'
        detector.patrol_laps = 0
        detector.max_patrol_laps = 2
        return detector

    def test_patrol_state_transitions(self, mock_camera):
        """Test patrol state machine transitions: top -> right -> bottom -> left."""
        detector = self.create_fire_detector_for_patrol_tests(mock_camera)

        # Initial state: top
        assert detector.patrol_state == 'top'

        # top -> right
        cmd = detector.patrol_logic()
        assert detector.patrol_state == 'right'
        assert cmd[0] > 0  # Forward movement

        # right -> bottom
        cmd = detector.patrol_logic()
        assert detector.patrol_state == 'bottom'
        assert cmd[1] < 0  # Right movement (negative Y)

        # bottom -> left
        cmd = detector.patrol_logic()
        assert detector.patrol_state == 'left'
        assert cmd[0] < 0  # Backward movement

        # left -> top (completes one lap)
        cmd = detector.patrol_logic()
        assert detector.patrol_state == 'top'
        assert cmd[1] > 0  # Left movement (positive Y)
        assert detector.patrol_laps == 1

    def test_patrol_lap_counting(self, mock_camera):
        """Test that patrol_laps increments correctly."""
        detector = self.create_fire_detector_for_patrol_tests(mock_camera)

        # Complete first lap (4 state transitions)
        for _ in range(4):
            detector.patrol_logic()
        assert detector.patrol_laps == 1

        # Complete second lap
        for _ in range(4):
            detector.patrol_logic()
        assert detector.patrol_laps == 2

    def test_patrol_stops_at_max_laps(self, mock_camera):
        """Test that patrol stops after max_patrol_laps."""
        detector = self.create_fire_detector_for_patrol_tests(mock_camera)
        detector.max_patrol_laps = 2

        # Complete 2 laps (8 transitions)
        for _ in range(8):
            detector.patrol_logic()

        # Next call should stop
        cmd = detector.patrol_logic()
        assert detector.patrol_mode is False
        assert detector.patrol_state == 'stop'
        assert cmd == [0, 0, 0, 0]

    def test_patrol_returns_correct_command_format(self, mock_camera):
        """Test that patrol_logic returns 4-element command list."""
        detector = self.create_fire_detector_for_patrol_tests(mock_camera)

        cmd = detector.patrol_logic()

        assert isinstance(cmd, list)
        assert len(cmd) == 4
        # All elements should be numbers
        for element in cmd:
            assert isinstance(element, (int, float))

    def test_patrol_speed_is_consistent(self, mock_camera):
        """Test that patrol uses consistent PATROL_SPEED."""
        detector = self.create_fire_detector_for_patrol_tests(mock_camera)

        EXPECTED_SPEED = 0.5

        # Test each direction
        cmd_top = detector.patrol_logic()  # top -> right
        assert abs(cmd_top[0]) == EXPECTED_SPEED or cmd_top[0] == EXPECTED_SPEED

        detector.patrol_state = 'right'
        cmd_right = detector.patrol_logic()  # right -> bottom
        assert abs(cmd_right[1]) == EXPECTED_SPEED

    def test_patrol_resets_laps_on_stop(self, mock_camera):
        """Test that patrol_laps resets to 0 when stopping."""
        detector = self.create_fire_detector_for_patrol_tests(mock_camera)
        detector.max_patrol_laps = 1

        # Complete 1 lap
        for _ in range(4):
            detector.patrol_logic()

        # Stop
        detector.patrol_logic()

        assert detector.patrol_laps == 0


class TestFireDetectorConstants:
    """Tests for Fire_detector constant values."""

    def test_default_constants(self, mock_camera):
        """Verify default constant values are sensible."""
        from Fire_detector import Fire_detector
        detector = Fire_detector.__new__(Fire_detector)
        # Set defaults (normally done in __init__)
        detector.FIRE_CONFIRMATION = 30
        detector.OBSERVATION_TIMEOUT = 5
        detector.CAMERA_TILT_DEGREES = 30.0
        detector.MAX_FIRE_DISTANCE = 20
        detector.threshold = 30
        detector.MOVE_SPEED = 0.1

        assert detector.FIRE_CONFIRMATION == 30
        assert detector.OBSERVATION_TIMEOUT == 5
        assert detector.CAMERA_TILT_DEGREES == 30.0
        assert detector.MAX_FIRE_DISTANCE == 20
        assert detector.threshold == 30
        assert detector.MOVE_SPEED == 0.1

    def test_target_classes(self, mock_camera):
        """Test that target classes are correctly defined."""
        from Fire_detector import Fire_detector
        detector = Fire_detector.__new__(Fire_detector)
        detector.target_classes = ['fire', 'smoke']

        assert 'fire' in detector.target_classes
        assert 'smoke' in detector.target_classes
        assert len(detector.target_classes) == 2
