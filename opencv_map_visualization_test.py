import cv2
import numpy as np
from pathlib import Path
import time




class MapViewer:
    def __init__(self, window_size=200):
        self.window_size = window_size
        file_path = Path("./ROAR/datasets/birds_eye_map.npy")
        self.main_map = np.load(file_path)

    def show_map(self, car_coords=None):
        if car_coords is not None:
            x_start = max(int(car_coords[0]) - int(self.window_size), 0)
            x_end = min(car_coords[0] + self.window_size, self.main_map.shape[1])
            y_start = max(car_coords[1] - self.window_size, 0)
            y_end = min(car_coords[1] + self.window_size, self.main_map.shape[0])

            zoomed_map = self.main_map[y_start:y_end, x_start:x_end]
        else:
            zoomed_map = self.main_map

        # Resize and display the zoomed_map using OpenCV
        dim = (500, 500)
        cv2.imshow("map", cv2.resize(zoomed_map, dim, interpolation=cv2.INTER_AREA))
        cv2.waitKey(0)
        cv2.destroyAllWindows()


map_viewer = MapViewer(window_size=200)


    


dx = 100
while dx > 0:
    MapViewer.show_map((3000-dx,3000),300)
    time.sleep(1)
    dx -= 10


