import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

file_name = 'rosbag2_2023_02_11-10_27_06'
df = pd.read_csv(f'/home/ubuntu/FYP-ROS/rosbag/data/data/{file_name}_data.csv')

gesture_model_file_name = 'rosbag2_2023_02_10-08_20_28'
df_gesture_model = pd.read_csv(f'/home/ubuntu/FYP-ROS/rosbag/data/data/{gesture_model_file_name}_data.csv')

# Create the figure
fig, axs = plt.subplots(3, 6, figsize=(20, 10))
acc_axes = axs[:, :3].ravel()
vel_axes = axs[:, 3:].ravel()
acc_data = [df[f'Imu{i}_linear_acceleration_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
vel_data = [df[f'Imu{i}_angular_velocity_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
acc_data_model = [df_gesture_model[f'Imu{i}_linear_acceleration_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
vel_data_model = [df_gesture_model[f'Imu{i}_angular_velocity_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
acc_titles = [f'Imu{i}_acc_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]
vel_titles = [f'Imu{i}_vel_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]

fig.suptitle(f'{file_name}')

# Store the last drawn lines
last_left_x_grp = [None] * 18
last_right_x_grp = [None] * 18
last_text = None

start_index = 0
end_index = 0
CROP_LENGTH = 50

last_text = fig.text(.97, .97, f'{start_index}-{end_index}({end_index-start_index})/{len(df)}', fontsize=15, horizontalalignment='right', verticalalignment='top')
plt.draw()

def draw_interval(start_index, end_index):
    for i, ax in enumerate(axs.ravel()):
        left_x = [start_index] * 2
        right_x = [int(end_index)] * 2
        y = np.linspace(-1.5*9.8, 1.5*9.8, 2)
        last_left_x_grp[i] = ax.plot(left_x, y, linestyle="dashed", color="red")
        last_right_x_grp[i] = ax.plot(right_x, y, linestyle="dashed", color="green")
    plt.draw()

def mouse_event(event):
    # Get the start and end x
    if event.xdata is None:
        return
    print(f'x: {event.xdata} and y: {event.ydata}')
    global start_index
    global end_index
    start_index = max(int(event.xdata), 0)
    end_index = min(start_index + CROP_LENGTH, len(df))

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

    draw_interval(start_index, end_index)
    last_text = fig.text(.97, .97, f'{start_index}-{end_index}({end_index-start_index})/{len(df)}', fontsize=15, horizontalalignment='right', verticalalignment='top')

def close_plt_and_save(event):
    global start_index
    global end_index
    # Save the cropped data
    df[start_index:end_index].to_csv(f'/home/ubuntu/FYP-ROS/rosbag/data/data_clean_50/{file_name}_data.csv', index=False)
    plt.close()
    print("Saved and closed")



# Add the save and quit button
btn_ax = fig.add_axes([0.9, 0.05, 0.07, 0.045])
bnext = Button(btn_ax, 'Save and Quit')
bnext.on_clicked(close_plt_and_save)

# bind enter key to save and quit
fig.canvas.mpl_connect('key_press_event', lambda event: [close_plt_and_save(event)] if event.key == 'enter' else [])

cid = fig.canvas.mpl_connect('button_press_event', mouse_event)

for ax, data, data_model, title in zip(acc_axes, acc_data, acc_data_model, acc_titles):
    ax.plot(df.index, data)
    ax.plot(df_gesture_model.index, data_model, color='gray', alpha=0.35)
    ax.set_title(title)
    ax.set_ylim([-1.5 * 9.8, 1.5 * 9.8])

for ax, data, data_model, title in zip(vel_axes, vel_data, vel_data_model, vel_titles):
    ax.plot(df.index, data)
    ax.plot(df_gesture_model.index, data_model, color='gray', alpha=0.35)
    ax.set_title(title)
    ax.set_ylim([-5, 5])

if len(df) <= CROP_LENGTH:
    start_index = 0
    end_index = len(df)
    draw_interval(start_index, end_index)

plt.show()

print(f'{file_name}')
print(f'{start_index}-{end_index}({end_index-start_index})/{len(df)}')