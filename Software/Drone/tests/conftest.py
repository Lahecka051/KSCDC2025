"""
Shared test fixtures for drone system tests.
"""
import pytest
import numpy as np
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class MockVideoCapture:
    """Mock OpenCV VideoCapture for testing without real cameras."""

    def __init__(self, width=640, height=640, frame=None):
        self._width = width
        self._height = height
        self._frame = frame if frame is not None else np.zeros((height, width, 3), dtype=np.uint8)
        self._is_opened = True

    def get(self, prop_id):
        """Mock cv2.CAP_PROP_FRAME_WIDTH (3) and CAP_PROP_FRAME_HEIGHT (4)"""
        if prop_id == 3:  # cv2.CAP_PROP_FRAME_WIDTH
            return self._width
        elif prop_id == 4:  # cv2.CAP_PROP_FRAME_HEIGHT
            return self._height
        return 0

    def read(self):
        """Return mock frame"""
        return self._is_opened, self._frame.copy()

    def isOpened(self):
        return self._is_opened

    def set_frame(self, frame):
        """Update the frame that will be returned by read()"""
        self._frame = frame

    def close(self):
        self._is_opened = False


class MockMavlinkConnection:
    """Mock pymavlink connection for testing without real drone hardware."""

    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = MockMav()
        self._messages = []

    def wait_heartbeat(self):
        return True

    def recv_match(self, type=None, blocking=True, timeout=None):
        """Return mock MAVLink messages"""
        if type == 'HEARTBEAT':
            return MockHeartbeat(armed=True)
        elif type == 'GLOBAL_POSITION_INT':
            return MockGlobalPositionInt(
                lat=37.5665 * 1e7,  # Seoul coordinates
                lon=126.9780 * 1e7,
                relative_alt=2000  # 2.0m in mm
            )
        elif type == 'VFR_HUD':
            return MockVfrHud(alt=2.0)
        return None


class MockMav:
    """Mock MAV object for sending commands."""

    def request_data_stream_send(self, *args):
        pass

    def set_mode_send(self, *args):
        pass

    def command_long_send(self, *args):
        pass

    def set_position_target_local_ned_send(self, *args):
        pass

    def set_position_target_global_int_send(self, *args):
        pass


class MockHeartbeat:
    """Mock HEARTBEAT message."""

    def __init__(self, armed=False):
        self.base_mode = 128 if armed else 0  # MAV_MODE_FLAG_SAFETY_ARMED = 128

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


class MockGlobalPositionInt:
    """Mock GLOBAL_POSITION_INT message."""

    def __init__(self, lat=375665000, lon=1269780000, relative_alt=2000):
        self.lat = lat
        self.lon = lon
        self.relative_alt = relative_alt  # in mm


class MockVfrHud:
    """Mock VFR_HUD message."""

    def __init__(self, alt=2.0):
        self.alt = alt


class MockYOLOModel:
    """Mock YOLO model for testing without real ML inference."""

    def __init__(self):
        self.names = {0: 'fire', 1: 'smoke'}
        self._detections = []

    def predict(self, frame, imgsz=640, conf=0.4, verbose=False):
        """Return mock detection results."""
        return [MockDetectionResult(self._detections)]

    def set_detections(self, detections):
        """Set what detections should be returned.

        Args:
            detections: List of (class_id, x1, y1, x2, y2, conf) tuples
        """
        self._detections = detections


class MockDetectionResult:
    """Mock YOLO detection result."""

    def __init__(self, detections):
        self.boxes = [MockBox(*d) for d in detections]


class MockBox:
    """Mock YOLO bounding box."""

    def __init__(self, class_id, x1, y1, x2, y2, conf):
        self.cls = [class_id]
        self.xyxy = [np.array([x1, y1, x2, y2])]
        self.conf = [conf]


# Pytest fixtures

@pytest.fixture
def mock_camera():
    """Provide a mock camera with default 640x640 black frame."""
    return MockVideoCapture()


@pytest.fixture
def mock_camera_with_red_dot():
    """Provide a mock camera with a red dot in the center."""
    import cv2
    frame = np.zeros((640, 640, 3), dtype=np.uint8)
    # Draw red circle in center
    cv2.circle(frame, (320, 320), 50, (0, 0, 255), -1)
    return MockVideoCapture(frame=frame)


@pytest.fixture
def mock_mavlink():
    """Provide a mock MAVLink connection."""
    return MockMavlinkConnection()


@pytest.fixture
def mock_yolo():
    """Provide a mock YOLO model."""
    return MockYOLOModel()


@pytest.fixture
def sample_gps_dict():
    """Provide sample GPS data as dictionary."""
    return {
        'lat': 37.5665,
        'lon': 126.9780,
        'alt': 2.0,
        'heading': 0
    }


@pytest.fixture
def sample_waypoints():
    """Provide sample patrol waypoints."""
    return [
        {'lat': 37.5665, 'lon': 126.9780},
        {'lat': 37.5670, 'lon': 126.9785},
        {'lat': 37.5675, 'lon': 126.9790},
    ]
