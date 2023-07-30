import cv2
import numpy as np
from pathlib import Path
import time
import keyboard




class MapViewer:
    def __init__(self, window_size=200):
        self.window_size = window_size
        file_path = Path("./ROAR/datasets/birds_eye_map.npy")
        self.main_map = np.load(file_path)

    def after_run_map(self, car_coords=[3000,3000]):



        #Resize and display the zoomed_map using OpenCV
        map_dim = (800, 500)
        text_dim = (800, 200)
        while True:

            #Calculates zoomed_map
            x_start = max(int(car_coords[0]) - int(self.window_size), 0)
            x_end = min(car_coords[0] + self.window_size, self.main_map.shape[1])
            y_start = max(car_coords[1] - self.window_size, 0)
            y_end = min(car_coords[1] + self.window_size, self.main_map.shape[0])      
            zoomed_map = self.main_map[y_start:y_end, x_start:x_end]

            #Resize zoomed map to dim
            zoomed_map = cv2.resize(zoomed_map, map_dim, interpolation=cv2.INTER_AREA)

            #WHY IS ZOOMED MAP A GRAY SCALE IMAGE 

            #Converts zoomed map 
            zoomed_map_color = np.zeros((zoomed_map.shape[0], zoomed_map.shape[1], 3), dtype=np.uint8)

            # Copy the grayscale zoomed_map into each channel of the color canvas
            zoomed_map_color[:, :, 0] = zoomed_map  # Blue channel (Channel 0)
            zoomed_map_color[:, :, 1] = zoomed_map  # Green channel (Channel 1)
            zoomed_map_color[:, :, 2] = zoomed_map  # Red channel (Channel 2)



            

            #Creates Discription canva
            description_canvas = np.zeros((800, 3200, 3), dtype=np.uint8)

            #putText()
            cv2.putText(description_canvas, "use AWSD to move around", (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 0), 2)
            #Resizes Description canva
            description_canvas = cv2.resize(description_canvas, text_dim, interpolation=cv2.INTER_AREA)
           
            #Combines Image
            combined_image = np.vstack((zoomed_map_color, description_canvas))

            

            cv2.imshow("map", combined_image )
            
            key = cv2.waitKey(1)
            if key == ord('a'):
                car_coords[0]-=5
            if key == ord('s'):
                car_coords[1]+=5
            if key == ord('d'):
                car_coords[0]+=5
            if key == ord('w'):
                car_coords[1]-=5
            if key == ord('p'):
                break


        

        

map_viewer = MapViewer(window_size=200)


    


map_viewer.show_map([3000,3000])




