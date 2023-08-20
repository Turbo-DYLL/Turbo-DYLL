import cv2
import numpy as np
from pathlib import Path
import util
import time
import math


class MapViewer:
    def __init__(self, window_size=200, checkpoint_path = Path("./ROAR/datasets/custom_checkpoints/default_checkpoints")):
        self.main_map = None
        self.window_size = window_size
        self.file_path = Path("./ROAR/datasets/birds_eye_map.npy")
        self.checkpoint_list = []
        with open(checkpoint_path, "r") as file:
            lines = file.readlines()
            for line in lines:
                coords = line.split(',')
                x_coord = float(coords[0].strip())
                y_coord = float(coords[2].strip())
                angle = float(coords[-1].strip())
                self.checkpoint_list.append([x_coord, y_coord,angle])
        
        self.checkpoint_amount = len(self.checkpoint_list)
        self.start_time = time.time()
        self.last_checkpoint_time = time.time()
        self.terminate = False
        self.next_checkpoint_number = 0
        self.recorded_times = [0.000 for _ in range(self.checkpoint_amount)]
        self.left_checkpoint = True


    def on_mouse_wheel(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEWHEEL:
            # Mouse wheel event detected
            scroll_amount = flags
            if self.window_size > 60:
                self.window_size -= int(flags/7864320 * 30) # Leo's Mouse wheel per tick modfiy for different sens
            else:
                if int(flags/7864320 * 30) < 0:
                    self.window_size -= int(flags/7864320 * 30) # <- prevents infinitely zoom that crashes the program
            # print(f"Mouse wheel scrolled: {scroll_amount}")

    def update(self, set_waypoint = None):

        """updates map with waypoint """
        self.main_map = np.load(self.file_path.__str__())
        self.main_map = self.main_map.astype(np.uint8)
        self.main_map = np.stack((self.main_map,) * 3, axis=-1) #converts it in to colored

        try:
            if set_waypoint != None:
                set_waypoint_array = util.get_coords_from_str_lines(set_waypoint)
                for coords in set_waypoint_array:
                    self.main_map[coords[1]-1:coords[1]+1, coords[0]-1:coords[0]+1] = (17, 36, 250) #<- blue lol
        except ValueError:
            pass

        
        # why the hell did bug fixxing this small section take me 2 hours - Leo 
        # (Coded with a boxful of monster and max volume dubstep)
    
    def checkpoint_check(self, car_coords):
        
        #convert angle to radians
        car_angle_radians = (270 - self.checkpoint_list[self.next_checkpoint_number][-1]) * (np.pi / 180)
        #find the start and end of the line
        #important to make sure that the car  is actually within the boundary of the x,y coords
        perpendicular_angle = car_angle_radians + np.pi / 2

        line_length = 190 #approx road length
        #TODO change line_length to the correct road length

        x1 = self.checkpoint_list[self.next_checkpoint_number][0] - line_length / 2 * np.cos(perpendicular_angle)
        y1 = self.checkpoint_list[self.next_checkpoint_number][1] - line_length / 2 * np.sin(perpendicular_angle)
        x2 = self.checkpoint_list[self.next_checkpoint_number][0] + line_length / 2 * np.cos(perpendicular_angle)
        y2 = self.checkpoint_list[self.next_checkpoint_number][1] + line_length / 2 * np.sin(perpendicular_angle)

        #conver checkpoint to point and slope
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
        
        perpendicular_distance = abs(m * car_coords[0] - car_coords[1] + b) / math.sqrt(m ** 2 + 1)
        #Gets the shortest distance between a line and a point

        distance_threshold = 1.5 #Adjust later for accuracy (basically carspeed/frame tick) @eric

        #very small optimization to skip 2 compares (short hand evaulation but im stupid)
        if perpendicular_distance >= distance_threshold:
            return False
        #check if the boundary is correct
        if min(x1, x2) <= car_coords[0] <= max(x1, x2):
            if min(y1, y2) <= car_coords[1] <= max(y1, y2):
                return True
        return False
        
    def add_checkpoint_to_map(self, car_x, car_y, car_angle_degree, checked = False):
 
        car_angle_radians = (270 - car_angle_degree) * (np.pi / 180)
        perpendicular_angle = car_angle_radians + np.pi / 2

        line_length = 50

        line_start_x = car_x - line_length / 2 * np.cos(perpendicular_angle)
        line_start_y = car_y - line_length / 2 * np.sin(perpendicular_angle)
        line_end_x = car_x + line_length / 2 * np.cos(perpendicular_angle)
        line_end_y = car_y + line_length / 2 * np.sin(perpendicular_angle)
        #generates the end point:


        line_color = (255, 0, 0) if not checked else (17, 250, 48) #adds the checkpoint line to the right color
        line_thickness = 2   
        
        line_start = (int(line_start_x), int(line_start_y))
        line_end = (int(line_end_x), int(line_end_y))

        cv2.line(self.main_map, line_start, line_end, line_color, line_thickness)

    def start_timer(self):
        self.start_time = time.time()

    def update_display_map(self, car_coords):
        """
        Displays an interactive Map that allows the tester to choose to save or discard the current run
        """

        #Resize and display the zoomed_map using OpenCV
        map_dim = (800, 600)
        text_dim = (800, 300)
        #Calculates zoomed_map
        x_start = max(int(car_coords[0]) - int(self.window_size * 4 / 3), 0) # *4/3 is to adjust for the change in dimension later
        x_end = min(int(car_coords[0]) + int(self.window_size * 4/3), self.main_map.shape[1])
        y_start = max(int(car_coords[1]) - self.window_size, 0)
        y_end = min(int(car_coords[1]) + self.window_size, self.main_map.shape[0])
        zoomed_map = self.main_map[y_start:y_end, x_start:x_end]

        #Resize zoomed map to dim
        zoomed_map = cv2.resize(zoomed_map, map_dim, interpolation=cv2.INTER_AREA)

        #Creates Discription canva
        description_canvas = np.zeros((1200, 3200, 3), dtype=np.uint8)

        #putText()
        time_elapsed = time.time() - self.start_time

        time_elapsed = np.round(time_elapsed, 3)

        #adding check points things here

        reached_checkpoint = self.checkpoint_check(car_coords)

        if reached_checkpoint and self.left_checkpoint:
            self.recorded_times[self.next_checkpoint_number] = np.round(time.time() - self.last_checkpoint_time,3)
            self.last_checkpoint_time = time.time()
            self.next_checkpoint_number += 1
            self.left_checkpoint = False
        else:
            self.left_checkpoint = True
        
    
        
        #renders checkpoints

        for i in range(self.checkpoint_amount):
            if i < self.next_checkpoint_number:
                self.add_checkpoint_to_map(self.checkpoint_list[i][0],self.checkpoint_list[i][1],self.checkpoint_list[i][2], checked = True)
            else:
                self.add_checkpoint_to_map(self.checkpoint_list[i][0],self.checkpoint_list[i][1],self.checkpoint_list[i][2], checked = False)
            

        cv2.putText(description_canvas, f'Total Time Elapsed: {time_elapsed}', (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 0), 3)
        self.horizontal_lines = 0
        self.vertical_lines = 1
        for i in range(self.checkpoint_amount):
            cv2.putText(description_canvas, f'Checkpoint {i+1}:  {self.recorded_times[i]}', (100 + self.horizontal_lines * 1000 , 100 + self.vertical_lines*150), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 0), 3)
            if self.horizontal_lines == 2:
                self.horizontal_lines = 0
                self.vertical_lines += 1
            else:
                self.horizontal_lines += 1

        description_canvas = cv2.resize(description_canvas, text_dim, interpolation=cv2.INTER_AREA)\
        
        #Combines Image
        combined_image = np.vstack((zoomed_map, description_canvas))

        #Show Combined Image
        cv2.imshow("map", combined_image )

        #Added Zoom function
        cv2.setMouseCallback("map", self.on_mouse_wheel)

        # Used to move the map
        key = cv2.waitKey(1)
                

        
                    


if __name__ == "__main__":
        x = 2542.809814453125
        y = 4069.600341796875
        waypoints_file = Path("./ROAR/datasets/segment_waypoint/eric-waypoints.txt")
        map_viewer = MapViewer(window_size=200)
        map_viewer.start_timer
        with open(waypoints_file, "r") as file:
            map_viewer.update(set_waypoint = file)
        map_viewer.update_display_map([x,y])
        for _ in range(2000):
            map_viewer.update_display_map([x,y])
            x -= 1
            y += 0.15
        time.sleep(1)




