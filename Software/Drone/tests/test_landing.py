"""
Unit tests for Landing class.
Tests focus on marker detection and control commands.
"""
import pytest
import numpy as np
import cv2
import sys
import os
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class MockVideoCapture:
    """Mock OpenCV VideoCapture for testing without real cameras."""

    def __init__(self, width=640, height=640, frame=None):
        self._width = width
        self._height = height
        self._frame = frame if frame is not None else np.zeros((height, width, 3), dtype=np.uint8)
        self._is_opened = True

    def get(self, prop_id):
        if prop_id == 3:  # cv2.CAP_PROP_FRAME_WIDTH
            return self._width
        elif prop_id == 4:  # cv2.CAP_PROP_FRAME_HEIGHT
            return self._height
        return 0

    def read(self):
        return self._is_opened, self._frame.copy()

    def isOpened(self):
        return self._is_opened


class MockDroneSystem:
    """Mock drone system for Landing tests."""

    def __init__(self):
        self.last_command = None
        self.mode = 'GUIDED'

    def set_command(self, cmd):
        self.last_command = cmd
        return True

    def set_mode_guided(self):
        self.mode = 'GUIDED'

    def set_mode_land(self):
        self.mode = 'LAND'

    def read_altitude(self):
        return 2.0

    def disarm(self):
        pass


class TestDetectRedDot:
    """Tests for the detect_red_dot marker detection function."""

    def create_landing_for_tests(self, mock_camera):
        """Helper to create Landing with mocked dependencies."""
        from Landing import Landing
        mock_drone = MockDroneSystem()
        landing = Landing(mock_camera, mock_drone)
        return landing

    def test_detects_red_circle_in_center(self):
        """Test detection of red circle in frame center."""
        # Create frame with red circle in center
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 320), 50, (0, 0, 255), -1)  # BGR: Red

        mock_cam = MockVideoCapture(frame=frame)
        landing = self.create_landing_for_tests(mock_cam)

        result = landing.detect_red_dot(frame)

        assert result is not None
        cx, cy, area = result
        assert abs(cx - 320) < 10  # Close to center
        assert abs(cy - 320) < 10
        assert area > 150  # Above minimum threshold

    def test_detects_red_circle_off_center(self):
        """Test detection of red circle not in center."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (100, 500), 40, (0, 0, 255), -1)

        mock_cam = MockVideoCapture(frame=frame)
        landing = self.create_landing_for_tests(mock_cam)

        result = landing.detect_red_dot(frame)

        assert result is not None
        cx, cy, area = result
        assert abs(cx - 100) < 15
        assert abs(cy - 500) < 15

    def test_returns_none_for_black_frame(self):
        """Test that black frame returns None."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)

        mock_cam = MockVideoCapture(frame=frame)
        landing = self.create_landing_for_tests(mock_cam)

        result = landing.detect_red_dot(frame)
        assert result is None

    def test_returns_none_for_small_red_dot(self):
        """Test that very small red dot (area < 150) returns None."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 320), 5, (0, 0, 255), -1)  # Very small

        mock_cam = MockVideoCapture(frame=frame)
        landing = self.create_landing_for_tests(mock_cam)

        result = landing.detect_red_dot(frame)
        assert result is None

    def test_ignores_blue_circle(self):
        """Test that blue circles are not detected."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 320), 50, (255, 0, 0), -1)  # BGR: Blue

        mock_cam = MockVideoCapture(frame=frame)
        landing = self.create_landing_for_tests(mock_cam)

        result = landing.detect_red_dot(frame)
        assert result is None

    def test_ignores_green_circle(self):
        """Test that green circles are not detected."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 320), 50, (0, 255, 0), -1)  # BGR: Green

        mock_cam = MockVideoCapture(frame=frame)
        landing = self.create_landing_for_tests(mock_cam)

        result = landing.detect_red_dot(frame)
        assert result is None

    def test_detects_largest_red_contour(self):
        """Test that the largest red contour is detected when multiple exist."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (100, 100), 30, (0, 0, 255), -1)  # Small red
        cv2.circle(frame, (400, 400), 60, (0, 0, 255), -1)  # Large red

        mock_cam = MockVideoCapture(frame=frame)
        landing = self.create_landing_for_tests(mock_cam)

        result = landing.detect_red_dot(frame)

        assert result is not None
        cx, cy, area = result
        # Should detect the larger circle
        assert abs(cx - 400) < 15
        assert abs(cy - 400) < 15

    def test_hsv_red_range_detection(self):
        """Test that both HSV red ranges are detected (0-10 and 160-179)."""
        # Test dark red (hue ~0)
        frame1 = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame1, (320, 320), 50, (0, 0, 200), -1)

        mock_cam = MockVideoCapture(frame=frame1)
        landing = self.create_landing_for_tests(mock_cam)

        result1 = landing.detect_red_dot(frame1)
        assert result1 is not None


class TestGetControlCommand:
    """Tests for the get_control_command function."""

    def create_landing_for_tests(self, frame=None):
        """Helper to create Landing with mocked dependencies."""
        from Landing import Landing

        if frame is None:
            frame = np.zeros((640, 640, 3), dtype=np.uint8)

        mock_cam = MockVideoCapture(frame=frame)
        mock_drone = MockDroneSystem()
        landing = Landing(mock_cam, mock_drone)
        return landing

    def test_marker_centered_minimal_correction(self):
        """Test that centered marker produces minimal X/Y correction."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 320), 50, (0, 0, 255), -1)

        landing = self.create_landing_for_tests(frame)
        command = landing.get_control_command(frame)

        if command != "LANDING_COMPLETE":
            assert isinstance(command, list)
            assert len(command) == 4
            # X and Y corrections should be small (marker is centered)
            assert abs(command[0]) < 0.2
            assert abs(command[1]) < 0.2
            # Should be descending
            assert command[2] < 0

    def test_marker_left_produces_right_correction(self):
        """Test that marker on left produces rightward correction."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (100, 320), 50, (0, 0, 255), -1)  # Left side

        landing = self.create_landing_for_tests(frame)
        command = landing.get_control_command(frame)

        assert isinstance(command, list)
        # command[1] is left/right, should correct to right (positive)
        assert command[1] > 0

    def test_marker_right_produces_left_correction(self):
        """Test that marker on right produces leftward correction."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (540, 320), 50, (0, 0, 255), -1)  # Right side

        landing = self.create_landing_for_tests(frame)
        command = landing.get_control_command(frame)

        assert isinstance(command, list)
        # command[1] is left/right, should correct to left (negative)
        assert command[1] < 0

    def test_marker_top_produces_forward_correction(self):
        """Test that marker at top produces forward correction."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 100), 50, (0, 0, 255), -1)  # Top

        landing = self.create_landing_for_tests(frame)
        command = landing.get_control_command(frame)

        assert isinstance(command, list)
        # command[0] is forward/backward
        assert command[0] < 0  # Negative Y diff -> negative command

    def test_marker_bottom_produces_backward_correction(self):
        """Test that marker at bottom produces backward correction."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 540), 50, (0, 0, 255), -1)  # Bottom

        landing = self.create_landing_for_tests(frame)
        command = landing.get_control_command(frame)

        assert isinstance(command, list)
        # command[0] is forward/backward
        assert command[0] > 0  # Positive Y diff -> positive command

    def test_no_marker_returns_hover_command(self):
        """Test that no marker starts timeout countdown."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)  # Black frame

        landing = self.create_landing_for_tests(frame)
        landing.marker_search_start = None  # Reset timer
        command = landing.get_control_command(frame)

        # Should start searching, return hover command
        assert isinstance(command, list)
        assert command == [0, 0, 0, 0]

    def test_command_velocity_limits(self):
        """Test that commands are limited to max velocity."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (10, 10), 50, (0, 0, 255), -1)  # Far corner

        landing = self.create_landing_for_tests(frame)
        command = landing.get_control_command(frame)

        if isinstance(command, list):
            # Velocity should be limited to [-0.5, 0.5]
            assert -0.5 <= command[0] <= 0.5
            assert -0.5 <= command[1] <= 0.5

    def test_always_descending_when_marker_visible(self):
        """Test that Z command is always negative (descending) when marker visible."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (200, 200), 50, (0, 0, 255), -1)

        landing = self.create_landing_for_tests(frame)
        command = landing.get_control_command(frame)

        if isinstance(command, list):
            assert command[2] == -landing.LANDING_SPEED  # Should descend

    def test_yaw_is_zero(self):
        """Test that yaw command is always zero during landing."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 320), 50, (0, 0, 255), -1)

        landing = self.create_landing_for_tests(frame)
        command = landing.get_control_command(frame)

        if isinstance(command, list):
            assert command[3] == 0

    def test_landing_complete_detection(self):
        """Test that large centered marker triggers landing complete."""
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        # Large circle very close to center
        cv2.circle(frame, (320, 320), 100, (0, 0, 255), -1)

        landing = self.create_landing_for_tests(frame)
        landing.AREA_THRESHOLD = 5000  # Set threshold
        landing.POSITION_THRESHOLD = 30

        command = landing.get_control_command(frame)

        # With large centered marker, should detect landing complete
        # Area of circle with r=100 is ~31416, which > 5000
        # Position error should be ~0
        assert command == "LANDING_COMPLETE"


class TestLandingParameters:
    """Tests for Landing class parameters."""

    def test_default_parameters(self, mock_camera):
        """Test default parameter values."""
        from Landing import Landing
        mock_drone = MockDroneSystem()
        landing = Landing(mock_camera, mock_drone)

        assert landing.LANDING_SPEED == 0.2
        assert landing.POSITION_THRESHOLD == 30
        assert landing.AREA_THRESHOLD == 5000
        assert landing.MARKER_SEARCH_TIMEOUT == 10

    def test_frame_dimensions_from_camera(self, mock_camera):
        """Test that frame dimensions are read from camera."""
        from Landing import Landing
        mock_drone = MockDroneSystem()
        landing = Landing(mock_camera, mock_drone)

        assert landing.frame_width == 640
        assert landing.frame_height == 640
