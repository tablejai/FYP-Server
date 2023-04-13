import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import mplcursors
import utils

SAMPLE_PER_GESTURE = 3

GESTURES = utils.GESTURES
colors_palette = utils.colors_palette

display_mask = {
    "STATIC": False,
    "SLIDE_UP": True,
    "SLIDE_DOWN": True,
    "SLIDE_LEFT": False,
    "SLIDE_RIGHT": False,
    "RELEASE": False,
    "GRASP": False,
    "NONE": False,
}

if __name__ == '__main__':
    dataset = utils.load_data()
    
    for gesture in [g for g in GESTURES if display_mask[g]]:
        print(f'{gesture}: {len(dataset[gesture])}')

    gesture_samples = []
    for gesture in [g for g in GESTURES if display_mask[g]]:
        for _ in range(SAMPLE_PER_GESTURE):
            gesture_samples.append((np.random.choice(dataset[gesture]), gesture))

    lines = []
    fig, axs = plt.subplots(3, 6, figsize=(20, 10))
    for i, (filename, gesture_type) in enumerate(gesture_samples):
        print(f'{filename} ({gesture_type})')
        raw_data = pd.read_csv(f'./rosbag/data/data_clean/{filename}_data.csv')
        raw_data['timestamp'] = raw_data['timestamp'] - raw_data['timestamp'][0]
        
        acc_axes = axs[:, :3].ravel()
        vel_axes = axs[:, 3:].ravel()
        acc_data = [raw_data[f'Imu{i}_linear_acceleration_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
        vel_data = [raw_data[f'Imu{i}_angular_velocity_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
        acc_titles = [f'Imu{i}_acc_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]
        vel_titles = [f'Imu{i}_vel_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]

        for ax, data, title in zip(acc_axes, acc_data, acc_titles):
            lines += ax.plot(data, label=f"{filename} ({gesture_type})", color=colors_palette[gesture_type])
            ax.set_title(title)
            ax.set_ylim([-1.5 * 9.8, 1.5 * 9.8])
            
        for ax, data, title in zip(vel_axes, vel_data, vel_titles):
            lines += ax.plot(data, label=f"{filename} ({gesture_type})", color=colors_palette[gesture_type])
            ax.set_title(title)
            ax.set_ylim([-5, 5])

    cursor = mplcursors.cursor(lines)
    @cursor.connect("add")
    def on_add(sel):
        sel.annotation.set_text(sel.artist.get_label())

    # get the unique colors in the plot
    colors_ = pd.unique([l.get_color() for l in lines])
    handles = [plt.Line2D([], [], color=c) for c in colors_]
    labels = [label for label in GESTURES if display_mask[label]]
    fig.legend(handles, labels)

    fig.suptitle(f'Heterogenity Analysis')
    plt.show()