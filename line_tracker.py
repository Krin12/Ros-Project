import cv2
import numpy as np

class LineTracker:
    def __init__(self):
        self._delta = 0.0
        self._delta2 = 0.0

    def process(self, img: np.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100]) 
        upper_yellow = np.array([30, 255, 250])
   
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = img.shape
        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)
        mask[0:h, 0:int(2 * w / 4)] = 0
        mask[0:h, int(2 * w / 4 + 25):w] = 0
        mask[int(h / 2 + 20):h,int(w / 2):int(w / 2+ 25)] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            err = cy - h / 2
            self._delta = err
        cv2.imshow("window", img)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)

    def detect_stop_line(self, img: np.ndarray) -> bool:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200]) 
        upper_white = np.array([180, 30, 255])

        stop_line_mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape
        search_top = int( h / 2 - 50 )
        search_bot = int(h)
        stop_line_mask[0:search_top, 0:w] = 0
        stop_line_mask[search_bot:h, 0:w] = 0
        M = cv2.moments(stop_line_mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            err = cy - h / 2
            self._delta = err
        cv2.imshow("window1", img)
        cv2.imshow("stop_line_mask", stop_line_mask)
        cv2.waitKey(3)

    @property
    def delta(self):
        return self._delta

def main():
    tracker = LineTracker()
    import time
    for i in range(100):
        img_path = '/home/ros2/Ros-Project/src/worlds/sample.png'
        img = cv2.imread(img_path)
        if img is None:
        	print(f"Error: Could not read image at {img_path}")
        	break
        tracker.process(img)
        tracker.detect_stop_line(img)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
