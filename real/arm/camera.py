"""Minimal OpenCV camera preview used by the real-arm loop."""


class Camera:
    def __init__(self, camera_id=0):
        if not isinstance(camera_id, int) or camera_id < 0:
            raise ValueError('camera ID must be a nonnegative integer')
        import cv2 as cv
        self.cv = cv
        self.window_name = 'Camera'
        self.capture = self.cv.VideoCapture(camera_id)
        if not self.capture.isOpened():
            self.capture.release()
            raise IOError(f'Cannot open camera {camera_id}')
        try:
            self.cv.namedWindow(self.window_name)
        except self.cv.error as exc:
            self.capture.release()
            raise IOError(f'Cannot create camera window: {exc}') from exc

    def poll(self):
        """Process one frame; False means Esc/window close requested exit."""
        try:
            ok, frame = self.capture.read()
            if not ok or frame is None:
                raise IOError('Cannot read camera frame')
            self.cv.imshow(self.window_name, frame)
            if self.cv.waitKey(1) & 0xFF == 27:
                return False
            return self.cv.getWindowProperty(self.window_name, self.cv.WND_PROP_VISIBLE) >= 1
        except self.cv.error as exc:
            raise IOError(f'Camera preview failed: {exc}') from exc

    def close(self):
        self.capture.release()
        try:
            self.cv.destroyWindow(self.window_name)
        except self.cv.error:
            pass  # The window may already have been closed.
