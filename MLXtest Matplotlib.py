#Test program for MLX90640 class, uses matplotlib for plotting temperatures
#Based on example from https://matplotlib.org/stable/gallery/animation/animation_demo.html
#UBC PHAS E-lab, Nov 2022
#Required Packages:
#pyserial
#matplotlib



from MLX90640 import MLX90640
import matplotlib
matplotlib.use('Tkagg')
import matplotlib.pyplot as plt
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
sensor = MLX90640('/dev/tty.usbmodem1203' , baud=115200, framerate=7)#Actual com port name will depend on system

fig, ax = plt.subplots()
plt.inferno()
loop = 0
try:
    while True:
        # Calculate temperature values from MLX RAM
        floatarray = np.array([[sensor.getCompensatedPixDataRAM(i+1, j+1) for i in range(24)] for j in range(32)])
        #floatarray = floatarray[2:][2:]
        floatarray = floatarray.T
        floatarray = gaussian_filter(floatarray, sigma=2)
        
        sections = 32  # Adjusted for the trimmed frame
        min_temp_for_tracking = 25  # Minimum temperature for tracking people
        max_temp_for_tracking = 37  # Maximum temperature to avoid artifacts
        
        # Reshape the array for section analysis
        reshaped_array = floatarray.reshape(floatarray.shape[0], sections, -1)
        
        # Calculate the mean temperature of each section
        section_means = reshaped_array.mean(axis=-1).mean(axis=0)
        
        # Apply both minimum and maximum temperature filters
        # Use np.logical_and for combined conditions
        valid_temps = np.logical_and(section_means > min_temp_for_tracking, section_means < max_temp_for_tracking)
        section_means_filtered = np.where(valid_temps, section_means, 0)
        
        # Find the index with the highest temperature that's still within human range
        max_section_index = np.argmax(section_means_filtered)
        max_temp = section_means_filtered[max_section_index]
        
        # Proceed if a valid heat source within the human temperature range is detected
        if max_temp > min_temp_for_tracking:
            center_section_index = sections // 2
            distance_from_center = max_section_index - center_section_index
            
            # Magnitude calculation adapted for the trimmed frame
            magnitude = int(abs(distance_from_center) / (sections // 2) * 255)
            magnitude = min(magnitude, 256)
            
            direction = 0 if distance_from_center < 0 else 1
            
            print(f"Direction: {direction}, Magnitude: {magnitude}")
            sensor.turn(direction, magnitude)
        else:
            print("No significant heat source detected.")
            
        # Visualization and loop maintenance
        cmap = ax.imshow(floatarray)
        ax.set_title("Temperature Map")
        cb = fig.colorbar(cmap, ax=ax)
        plt.pause(0.001)
        sensor.updateRAM()
        loop += 1
        print(loop)
        cb.remove()
        ax.cla()
        
finally:
    sensor.close()
