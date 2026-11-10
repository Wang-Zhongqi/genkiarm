"""OpenCV camera preview, standalone or used by the real-arm loop.

python real/arm/camera.py --camera-id 0
"""

import argparse
import sys


class Camera:
    def __init__(self, camera_id=0):
        if not isinstance(camera_id, int) or camera_id < 0:
            raise ValueError('camera ID must be a nonnegative integer')
        import cv2 as cv
        self.cv = cv
        self.window_name = 'Camera'
        self.roi_window_name = 'Camera ROI'
        self.frame_size = None
        self.drag_start = None
        self.drag_end = None
        self.roi = None
        self.roi_window_open = False
        self.capture = self.cv.VideoCapture(camera_id)
        if not self.capture.isOpened():
            self.capture.release()
            raise IOError(f'Cannot open camera {camera_id}')
        try:
            self.cv.namedWindow(self.window_name)
            self.cv.setMouseCallback(self.window_name, self._on_mouse)
        except self.cv.error as exc:
            self.capture.release()
            raise IOError(f'Cannot create camera window: {exc}') from exc

    def _selection_bounds(self):
        if self.drag_start is None or self.drag_end is None:
            return None
        x1, x2 = sorted((self.drag_start[0], self.drag_end[0]))
        y1, y2 = sorted((self.drag_start[1], self.drag_end[1]))
        return (x1, y1, x2, y2) if x2 > x1 and y2 > y1 else None

    def _close_roi(self):
        if self.roi_window_open:
            try:
                self.cv.destroyWindow(self.roi_window_name)
            except self.cv.error:
                pass
            self.roi_window_open = False

    def _on_mouse(self, event, x, y, flags, userdata):
        if self.frame_size is None:
            return
        width, height = self.frame_size
        point = (max(0, min(x, width)), max(0, min(y, height)))
        if event == self.cv.EVENT_RBUTTONDOWN:
            self.roi = self.drag_start = self.drag_end = None
            self._close_roi()
        elif event == self.cv.EVENT_LBUTTONDOWN:
            self.drag_start = self.drag_end = point
        elif event == self.cv.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_end = point
        elif event == self.cv.EVENT_LBUTTONUP and self.drag_start is not None:
            self.drag_end = point
            selection = self._selection_bounds()
            if selection is not None:
                self.roi = selection
            self.drag_start = self.drag_end = None

    def poll(self):
        """Process one frame; False means Esc/window close requested exit."""
        try:
            ok, frame = self.capture.read()
            if not ok or frame is None:
                raise IOError('Cannot read camera frame')
            height, width = frame.shape[:2]
            self.frame_size = (width, height)
            if self.roi_window_open:
                try:
                    visible = self.cv.getWindowProperty(
                        self.roi_window_name, self.cv.WND_PROP_VISIBLE,
                    ) >= 1
                except self.cv.error:
                    visible = False
                if not visible:
                    self.roi = None
                    self._close_roi()
            if self.roi is not None:
                x1, y1, x2, y2 = self.roi
                x1, x2 = min(x1, width), min(x2, width)
                y1, y2 = min(y1, height), min(y2, height)
                if x2 > x1 and y2 > y1:
                    if not self.roi_window_open:
                        self.cv.namedWindow(self.roi_window_name, self.cv.WINDOW_NORMAL)
                        scale = min(1, 640 / (x2 - x1), 480 / (y2 - y1))
                        self.cv.resizeWindow(self.roi_window_name,
                                             max(1, round((x2 - x1) * scale)),
                                             max(1, round((y2 - y1) * scale)))
                        self.roi_window_open = True
                    self.cv.imshow(self.roi_window_name, frame[y1:y2, x1:x2])
                else:
                    self.roi = None
                    self._close_roi()
            selection = self._selection_bounds() if self.drag_start is not None else self.roi
            display = frame
            if selection is not None:
                display = frame.copy()
                x1, y1, x2, y2 = selection
                self.cv.rectangle(display, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 0), 2)
            self.cv.imshow(self.window_name, display)
            if self.cv.waitKey(1) & 0xFF == 27:
                return False
            return self.cv.getWindowProperty(self.window_name, self.cv.WND_PROP_VISIBLE) >= 1
        except self.cv.error as exc:
            raise IOError(f'Camera preview failed: {exc}') from exc

    def close(self):
        self.capture.release()
        self._close_roi()
        try:
            self.cv.destroyWindow(self.window_name)
        except self.cv.error:
            pass  # The window may already have been closed.


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-id', type=int, default=0,
                        help='Camera index (default: 0)')
    args = parser.parse_args(argv)
    if args.camera_id < 0:
        parser.error('--camera-id must be nonnegative')

    camera = None
    try:
        camera = Camera(args.camera_id)
        print(f'Camera {args.camera_id} opened. Press Esc or close the window to exit.')
        while camera.poll():
            pass
    except KeyboardInterrupt:
        return 130
    except (OSError, ImportError) as exc:
        print(f'Camera preview failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if camera is not None:
            camera.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
