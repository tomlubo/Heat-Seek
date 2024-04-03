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
sensor = MLX90640('/dev/tty.usbmodem21203' , baud=115200, framerate=7)#Actual com port name will depend on system

fig, ax = plt.subplots()
plt.inferno()
loop = 0
try:
    while True:
        # Calculate temperature values from MLX RAM
        floatarray = np.array([[sensor.getCompensatedPixDataRAM(i+1, j+1) for i in range(24)] for j in range(32)])
        floatarray = np.rot90(floatarray, 2).T
        floatarray = gaussian_filter(floatarray, sigma=2)
        
        sections = 16  # Number of vertical sections
        screen_width_in_sections = sections 
        
        # Reshape the array so each row represents two columns (a section)
        # Assuming the floatarray shape is consistent with the number of sections
        reshaped_array = floatarray.reshape(floatarray.shape[0], sections, -1)
        
        # Calculate the mean temperature of each section
        section_means = reshaped_array.mean(axis=-1).mean(axis=0)
        
        # Apply the minimum temperature filter
        section_means_filtered = np.where(section_means > 20, section_means, 0)
        
        # Find the index of the section with the highest temperature above the threshold
        max_section_index = np.argmax(section_means_filtered)
        max_temp = section_means_filtered[max_section_index]
        
        # Check if a valid heat source was detected
        if max_temp > 20:
            # Determine the center section index
            center_section_index = screen_width_in_sections // 2
            
            # Calculate distance from the center section
            distance_from_center = max_section_index - center_section_index
            
            # Map the distance from the center to magnitude range [0, 256]
            # The farthest section from the center on either side should correspond to magnitude 256
            # Linearly scale distance to this range
            max_distance = screen_width_in_sections // 2  # Maximum possible distance from the center
            magnitude = int(abs(distance_from_center) / max_distance * 255)  
            
            # Ensure magnitude does not exceed 256 due to rounding
            magnitude = min(magnitude, 256)
            
            # Determine direction: 0 for left, 1 for right, based on the sign of distance_from_center
            direction = 0 if distance_from_center < 0 else 1
            
            # Invoke the turn function with direction and magnitude
            print(f"Direction: {direction}, Magnitude: {magnitude}")  # Debug print
            sensor.turn(direction, magnitude)
        else:
            # No significant heat source detected
            magnitude = 0
            print("No significant heat source detected.")
        
                
        # Debugging print statement
        #print(f'Section: {max_section_index}, Temp: {max_temp}, Direction: {"None" if direction is None else ("Left" if direction == 0 else "Right")}, Magnitude: {magnitude}')

        # Visualization and loop maintenance
        cmap = ax.imshow(floatarray)  # Show the image
        ax.set_title("Temperature Map")
        cb = fig.colorbar(cmap, ax=ax)  # Show a colorbar
        plt.pause(0.001)
        sensor.updateRAM()  # Get a new copy of RAM from MLX90640
        loop += 1
        print(loop)
        cb.remove()  # Remove old plots
        ax.cla()
        
finally:
    sensor.close()
