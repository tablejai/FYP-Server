import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import mplcursors
import os
os.sys.path.append('/home/ubuntu/FYP-Glove/src/utils/utils')
from data_loading import load_data
import utils

GESTURES = utils.GESTURES
colors_palette = utils.colors_palette
target_gesture = GESTURES[1]

SAMPLE_PER_GESTURE = 4

if __name__ == '__main__':
    dataset, cnt = load_data()
    gesture_samples = dataset[target_gesture]
    
    # sample random 10 samples
    gesture_samples = np.random.choice(gesture_samples, SAMPLE_PER_GESTURE)

    fig, axs = plt.subplots(3, 6, figsize=(15, 8))
    lines = []
    for i, filename in enumerate(gesture_samples):
        print(f"{filename=}")
        raw_data = pd.read_csv(f'./rosbag/data/data_clean_50/{filename}_data.csv')
        raw_data['timestamp'] = raw_data['timestamp'] - raw_data['timestamp'][0]
        
        acc_axes = axs[:, :3].ravel()
        vel_axes = axs[:, 3:].ravel()
        acc_data = [raw_data[f'Imu{i}_linear_acceleration_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
        vel_data = [raw_data[f'Imu{i}_angular_velocity_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
        acc_titles = [f'Imu{i}_acc_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]
        vel_titles = [f'Imu{i}_vel_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]

        for ax, data, title in zip(acc_axes, acc_data, acc_titles):
            lines += ax.plot(data, label=filename)
            ax.set_title(title)
            ax.set_ylim([-1.5 * 9.8, 1.5 * 9.8])
            
        for ax, data, title in zip(vel_axes, vel_data, vel_titles):
            lines += ax.plot(data, label=filename)
            ax.set_title(title)
            ax.set_ylim([-5, 5])

    cursor = mplcursors.cursor(lines, hover=True)
    @cursor.connect("add")
    def on_add(sel):
        sel.annotation.set_text(sel.artist.get_label())

    fig.suptitle(f'Homogeneity Analysis ({target_gesture})')

    colors_ = np.unique([l.get_color() for l in lines])
    handles = [plt.Line2D([], [], color=c) for c in colors_]
    labels = np.unique([l.get_label() for l in lines])
    fig.legend(handles, labels, loc='upper right')
    plt.show()