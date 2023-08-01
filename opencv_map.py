import cv2
import numpy as np
from pathlib import Path
import util




class MapViewer:
    def __init__(self, window_size=200):
        self.window_size = window_size
        self.file_path = Path("./ROAR/datasets/birds_eye_map.npy")
        
        self.main_map = np.load(self.file_path)
        self.main_map = np.stack((self.main_map,) * 3, axis=-1)

        
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
    def update(self, set_waypoint = None, current_waypoint = None):
        """updates map with waypoint """
        self.main_map = np.load(self.file_path)
        self.main_map = self.main_map.astype(np.uint8)
        self.main_map = np.stack((self.main_map,) * 3, axis=-1) #converts it in to colored

        if set_waypoint != None:
            set_waypoint_array = util.extract_xy_coords_from_string(set_waypoint)
            for coords in set_waypoint_array: 
                self.main_map[coords[1]-1:coords[1]+1, coords[0]-1:coords[0]+1] = (17, 36, 250) #<- blue lol

        if current_waypoint != None:
            current_waypoint_array = util.extract_xy_coords_from_array(current_waypoint)
            for coords in current_waypoint_array: 
                self.main_map[coords[1]-1:coords[1]+1, coords[0]-1:coords[0]+1] = (17, 250, 48) #<- green haha
        # why the hell did bug fixxing this small section take me 2 hours - Leo 
        # (Coded with a boxful of monster and max volume dubstep)


    def interactive_map(self, car_coords=[3000,3000]):
        """
        Displays an interactive Map that allows the tester to choose to save or discard the current run
        """

        
        #Resize and display the zoomed_map using OpenCV
        map_dim = (800, 600)
        text_dim = (800, 200)
        while True:

            #Calculates zoomed_map
            x_start = max(int(car_coords[0]) - int(self.window_size * 4 / 3), 0) # *4/3 is to adjust for the change in dimension later
            x_end = min(int(car_coords[0]) + int(self.window_size * 4/3), self.main_map.shape[1])
            y_start = max(int(car_coords[1]) - self.window_size, 0)
            y_end = min(int(car_coords[1]) + self.window_size, self.main_map.shape[0])      
            zoomed_map = self.main_map[y_start:y_end, x_start:x_end]

            #Resize zoomed map to dim
            zoomed_map = cv2.resize(zoomed_map, map_dim, interpolation=cv2.INTER_AREA)

            #WHY IS NORMAL MAP A GRAY SCALE IMAGE - Leo

            # #Converts zoomed map 
            # zoomed_map_color = np.zeros((zoomed_map.shape[0], zoomed_map.shape[1], 3), dtype=np.uint8)

            # # Copy the grayscale zoomed_map into each channel of the color canvas
            # zoomed_map_color[:, :, 0] = zoomed_map  # Blue channel (Channel 0)
            # zoomed_map_color[:, :, 1] = zoomed_map  # Green channel (Channel 1)
            # zoomed_map_color[:, :, 2] = zoomed_map  # Red channel (Channel 2)

            #Creates Discription canva
            description_canvas = np.zeros((800, 3200, 3), dtype=np.uint8)

            #putText()
            cv2.putText(description_canvas, "use AWSD to move around    Use Scroll Wheel to zoom", (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 0), 3)
            cv2.putText(description_canvas, "Press 'P' to save  Press 'L' to discard", (100, 400), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 0), 3)
            cv2.putText(description_canvas, "Have fun testing - Leo", (2000, 600), cv2.FONT_HERSHEY_SIMPLEX, 3, (191, 64, 191), 3)
            cv2.putText(description_canvas, "Green is current run   Blue is saved waypoint", (100, 600), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 0), 3)
            #Resizes Description canva
            description_canvas = cv2.resize(description_canvas, text_dim, interpolation=cv2.INTER_AREA)
           
            #Combines Image
            combined_image = np.vstack((zoomed_map, description_canvas))

            #Show Combined Image
            cv2.imshow("map", combined_image )

            #Added Zoom function
            cv2.setMouseCallback("map", self.on_mouse_wheel)

            # Used to move the map
            key = cv2.waitKey(1)
            if key == ord('a'):
                car_coords[0]-=20
            if key == ord('s'):
                car_coords[1]+=20
            if key == ord('d'):
                car_coords[0]+=20
            if key == ord('w'):
                car_coords[1]-=20
            #Save
            if key == ord('p'):
                cv2.destroyAllWindows()
                return 0
            #Discard
            if key == ord('l'):
                cv2.destroyAllWindows()
                return 1



        

        
if __name__ == "__main__":
    # map_viewer = MapViewer(window_size=200)
    # map_viewer.interactive_map([3000,3000])
    print(self.size)




