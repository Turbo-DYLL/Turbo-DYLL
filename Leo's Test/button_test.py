import matplotlib.pyplot as plt

def button1_clicked(event):
    print("Button 1 clicked!")
    plt.close()
    return True

def button2_clicked(event):
    print("Button 2 clicked!")
    plt.close()
    return False

def create_buttons():
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.2)

    button1_ax = plt.axes([0.2, 0.1, 0.2, 0.075])
    button1 = plt.Button(button1_ax, "Save", color = '#8BC34A')
    button1.on_clicked(button1_clicked)

    button2_ax = plt.axes([0.6, 0.1, 0.2, 0.075])
    button2 = plt.Button(button2_ax, "Discard", color = '#FF5252')
    button2.on_clicked(button2_clicked)

    plt.show()

if __name__ == "__main__":
    result = create_buttons()
    print("The result is:", result)
