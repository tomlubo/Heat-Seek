#Test program for MLX90640 class, uses matplotlib for plotting temperatures
#Based on example from https://matplotlib.org/stable/gallery/animation/animation_demo.html
#UBC PHAS E-lab, Nov 2022
#Required Packages:
#pyserial
#matplotlib



from MLX90640 import MLX90640
# import matplotlib
# matplotlib.use('Tkagg')
# import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter

#MLX Framerate values 0-7 are 0.5-64Hz
#0 = 0.5Hz
#1 = 1Hz
#2 = 2Hz
#3 = 4Hz
#4 = 8Hz
#5 = 16Hz
#6 = 32Hz
#7 = 64Hz
sensor = MLX90640('/dev/tty.usbmodem21203' , baud=115200, framerate=7)#Actual com port name will depend on system

# fig, ax = plt.subplots()
# plt.inferno()
loop = 0
try:
    while True:
        # Calculate temperature values from MLX RAM
        floatarray = np.array([[sensor.getCompensatedPixDataRAM(i+1, j+1) for i in range(24)] for j in range(32)])
        floatarray = np.rot90(floatarray, 2).T
        floatarray = gaussian_filter(floatarray, sigma=2)
        
        sections =16  # Number of vertical sections
        min_temp_for_tracking = 20  # Minimum temperature for tracking (in Celsius)
        
        # Reshape the array so each row represents two columns (a section)
        # Assuming the floatarray shape is consistent with the number of sections
        reshaped_array = floatarray.reshape(floatarray.shape[0], sections, -1)
        
        # Calculate the mean temperature of each section
        section_means = reshaped_array.mean(axis=-1).mean(axis=0)
        
        # Apply the minimum temperature filter
        section_means_filtered = np.where(section_means > min_temp_for_tracking, section_means, 0)
        
        # Find the index of the section with the highest temperature above the threshold
        max_section_index = np.argmax(section_means_filtered)
        max_temp = section_means_filtered[max_section_index]
        
        # Check if a valid heat source was detected
        if max_temp > 10:
            # Middle sections where the robot does not turn
            middle_sections = [7, 8]  # Adjust based on your screen division and preference

            #print(max_section_index in middle_sections)
            if max_section_index in middle_sections:
                direction = None
                magnitude = 0
                #print(True)
            else:
                
                direction = 0 if max_section_index < middle_sections[0] else 1
                magnitude = abs(max_section_index - (sections))  # Calculate magnitude
                #print(direction,magnitude)
                sensor.turn(direction, magnitude)

        
        # Debugging print statement
        #print(f'Section: {max_section_index}, Temp: {max_temp}, Direction: {"None" if direction is None else ("Left" if direction == 0 else "Right")}, Magnitude: {magnitude}')

        # Visualization and loop maintenance
        # cmap = ax.imshow(floatarray)  # Show the image
        # ax.set_title("Temperature Map")
        # cb = fig.colorbar(cmap, ax=ax)  # Show a colorbar
        # plt.pause(0.001)
        sensor.updateRAM()  # Get a new copy of RAM from MLX90640
        loop += 1
        print(loop)
        # cb.remove()  # Remove old plots
        # ax.cla()
        
finally:
    sensor.close()
