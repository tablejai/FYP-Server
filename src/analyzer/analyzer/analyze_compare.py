import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import utils

# Load the data
dataset = utils.load_data()
target_gesture = "SLIDE_UP"
target_data = dataset[target_gesture]
num_data = len(target_data)
print(f"Number of data samples for {target_gesture}: {num_data}")

# Define helper functions
def plot_data(ax_list, data_list, title_list, color, linestyle="solid"):
    lines = []
    for ax, data, title in zip(ax_list, data_list, title_list):
        lines += ax.plot(data, color=color, linestyle=linestyle)
        ax.set_title(title)
        ax.set_ylim([-1.5 * 9.8, 1.5 * 9.8] if 'acc' in title else [-5, 5])
    return lines

# Create the figure and axes
fig, axs = plt.subplots(3, 6, figsize=(20, 10))
acc_axes = axs[:, :3].ravel()
vel_axes = axs[:, 3:].ravel()
acc_titles = [f'Imu{i}_acc_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]
vel_titles = [f'Imu{i}_vel_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]

# Plot the first reference data
file_path1 = target_data[0]
raw_data1 = pd.read_csv(f'./rosbag/data/data_clean/{file_path1}_data.csv')
raw_data1['timestamp'] = raw_data1['timestamp'] - raw_data1['timestamp'][0]

acc_data1 = [raw_data1[f'Imu{i}_linear_accleration_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
vel_data1 = [raw_data1[f'Imu{i}_angular_velocity_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]

plot_data(acc_axes, acc_data1, acc_titles, 'green')
plot_data(vel_axes, vel_data1, vel_titles, 'green')
fig.legend([f'{file_path1}'])

# Define the function to update the plot
plot_index = 1
lines = []
def next_plot(event):
    global file_path1
    global plot_index
    global lines
    
    if event.key != 'enter':
        return
    
    # Remove the reference lines
    for line in lines:
        line.remove()
    lines = []
    
    # Plot the next data
    file_path2 = target_data[plot_index]
    raw_data2 = pd.read_csv(f'./rosbag/data/data_clean/{file_path2}_data.csv')
    raw_data2['timestamp'] = raw_data2['timestamp'] - raw_data2['timestamp'][0]

    acc_data2 = [raw_data2[f'Imu{i}_linear_accleration_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
    vel_data2 = [raw_data2[f'Imu{i}_angular_velocity_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]

    lines = plot_data(acc_axes, acc_data2, acc_titles, "red", linestyle="dashed")
    lines += plot_data(vel_axes, vel_data2, vel_titles, "red", linestyle="dashed")
    plot_index +=1

    fig.legend([f'{file_path1}', f'{file_path2}'])
    fig.suptitle(f'Compare({target_gesture}): {file_path1} <-> {file_path2} ')
    plt.draw()

plt.connect('key_press_event', next_plot)
plt.show()