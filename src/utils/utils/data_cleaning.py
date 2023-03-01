import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

file_name = 'rosbag2_2023_02_11-09_09_53_data' 
df = pd.read_csv(f'/home/ubuntu/FYP-ROS/rosbag/data/data/{file_name}.csv')

# Create the figure
fig, axs = plt.subplots(3, 6, figsize=(20, 10))
acc_axes = axs[:, :3].ravel()
vel_axes = axs[:, 3:].ravel()
acc_data = [df[f'Imu{i}_linear_accleration_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
vel_data = [df[f'Imu{i}_angular_velocity_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
acc_titles = [f'Imu{i}_acc_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]
vel_titles = [f'Imu{i}_vel_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]

fig.suptitle(f'{file_name}')

# Store the last drawn lines
last_left_x_grp = [None] * 18
last_right_x_grp = [None] * 18
last_text = None

def mouse_event(event):
    # Get the start and end x
    if event.xdata is None:
        return
    print(f'x: {event.xdata} and y: {event.ydata}')
    start_index = int(event.xdata)
    end_index = min(start_index + 100, len(df))

    # Remove the last drawn lines
    global last_left_x_grp
    global last_right_x_grp
    global last_text
    for last_left_x, last_right_x in zip(last_left_x_grp, last_right_x_grp):
        if last_left_x is not None:
            last_left_x.pop(0).remove()
        if last_right_x is not None:
            last_right_x.pop(0).remove()

    if last_text is not None:
        last_text.remove()

    # Draw the new lines
    for i, ax in enumerate(axs.ravel()):
        left_x = [start_index] * 2
        right_x = [int(end_index)] * 2
        y = np.linspace(-1.5*9.8, 1.5*9.8, 2)
        last_left_x_grp[i] = ax.plot(left_x, y, linestyle="dashed", color="red")
        last_right_x_grp[i] = ax.plot(right_x, y, linestyle="dashed", color="green")
    
    last_text = fig.text(.97, .97, f'{start_index}-{end_index}/{len(df)}', fontsize=15, horizontalalignment='right', verticalalignment='top')
    plt.draw()

    # Save the cropped data
    df[start_index:end_index].to_csv(f'/home/ubuntu/FYP-ROS/rosbag/data/data_cropped/{file_name}.csv', index=False)

cid = fig.canvas.mpl_connect('button_press_event', mouse_event)

for ax, data, title in zip(acc_axes, acc_data, acc_titles):
    ax.plot(df.index, data)
    ax.set_title(title)
    ax.set_ylim([-1.5 * 9.8, 1.5 * 9.8])

for ax, data, title in zip(vel_axes, vel_data, vel_titles):
    ax.plot(df.index, data)
    ax.set_title(title)
    ax.set_ylim([-5, 5])

plt.show()