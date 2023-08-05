import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from pathlib import Path
class ImageWithButtons:
    def __init__(self, image_file):
        # Load the image data from the npy file
        self.image_data = np.load(Path("./ROAR/datasets/birds_eye_map.npy"))

        self.button_clicked = None

    def button1_clicked(self, event):
        print("Button 1 clicked!")
        self.button_clicked = "Button 1"
        plt.close()

    def button2_clicked(self, event):
        print("Button 2 clicked!")
        self.button_clicked = "Button 2"
        plt.close()

    def create_plot(self):
        fig, ax = plt.subplots(figsize=(8, 6))  # Adjust the figure size here (in inches)

        # Display the image using imshow in the subplot
        ax.imshow(self.image_data, cmap='gray')
        ax.set_title("Image")
        ax.axis('off')  # Turn off x and y axes

        # Add buttons below the image
        button1_ax = plt.axes([0.3, 0.05, 0.2, 0.075])
        button1 = plt.Button(button1_ax, "Button 1")
        button1.on_clicked(self.button1_clicked)

        button2_ax = plt.axes([0.55, 0.05, 0.2, 0.075])
        button2 = plt.Button(button2_ax, "Button 2")
        button2.on_clicked(self.button2_clicked)

        plt.show()

if __name__ == "__main__":
    image_file_path = "path/to/your/image.npy"
    image_with_buttons = ImageWithButtons(image_file_path)
    image_with_buttons.create_plot()

    print("Button clicked:", image_with_buttons.button_clicked)
