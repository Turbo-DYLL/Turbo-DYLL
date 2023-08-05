import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

class button_click_detector:
    """Create Buttons i guess"""
    #note: 
    # button1 = save
    # button2 = discard
    # button3 = terminate

    def __init__(self):
        self.button_clicked = None
        self.main_map_array = np.load(Path("./ROAR/datasets/birds_eye_map.npy"))

        # trim the useless part of the map
        region_of_interest = (1000, 6000, 2300, 6000)
        x_start, x_end, y_start, y_end = region_of_interest
        self.main_map_array = self.main_map_array[y_start:y_end, x_start:x_end]

    def button1_clicked(self, event):
        print("Button 1 (Save) clicked!")
        self.button_clicked = 1
        plt.close()

    def button2_clicked(self, event):
        print("Button 2 (Discard) clicked!")
        self.button_clicked = 2
        plt.close()
    
    def button3_clicked(self, event):
        print("Button 2 (False) clicked!")
        self.button_clicked = 2
        plt.close()

    def create_buttons(self):
        fig, ax = plt.subplots(1, 2, figsize=(14, 6))
        plt.subplots_adjust(bottom=0.2)
        
        ax[0].imshow(self.main_map_array, cmap='cool') #cool cause i am :)
        ax[0].set_title("this is blue because I didnt add the rest yet :)")
        ax[0].axis('off')

        # Display the "Save" button
        button1_ax = plt.axes([0.2, 0.1, 0.2, 0.075])
        button1 = plt.Button(button1_ax, "Save", color='#6FCF97') # <- soft green :)
        button1.on_clicked(self.button1_clicked)

        # Display the "discard" button
        button2_ax = plt.axes([0.6, 0.1, 0.2, 0.075])
        button2 = plt.Button(button2_ax, "Discard", color='#E57373') # <- soft red >:)
        button2.on_clicked(self.button2_clicked)

        #display the current map on top of discard button 
        ax[1].imshow(self.main_map_array, cmap='gray')
        ax[1].set_title("Full Mini Map")
        ax[1].axis('off')

        #plt.get_current_fig_manager().full_screen_toggle() #Full screen that just blocks everything :/
        plt.show()


#test code
# test =  button_click_detector()
# test.create_buttons()

# print("The result is:", test.button_clicked)
