import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

GESTURE = [
    "STATIC",
    "SLIDE_UP",
    "SLIDE_DOWN",
    "SLIDE_LEFT",
    "SLIDE_RIGHT",
    "ZOOM_IN",
    "ZOOM_OUT",
    # "HIGHLIGHT",
    # "ON_YES",
    # "OFF_NO",
    # "END"
]

def transform_to_relative_time(data):
    data['timestamp'] = data['timestamp'] - data['timestamp'][0]
    return data

def analyze_gesture():
    dataset = {
        GESTURE[0]: ["rosbag2_2023_02_10-06_45_38", "rosbag2_2023_02_10-06_46_56", "rosbag2_2023_02_10-06_47_17", "rosbag2_2023_02_11-09_01_37", "rosbag2_2023_02_11-09_03_05", "rosbag2_2023_02_11-09_03_14", ], 
        GESTURE[1]: ["rosbag2_2023_02_10-07_49_15", "rosbag2_2023_02_10-07_49_27", "rosbag2_2023_02_10-07_50_35", "rosbag2_2023_02_11-09_08_13", "rosbag2_2023_02_11-09_08_28", "rosbag2_2023_02_11-09_08_39", ],
        GESTURE[2]: ["rosbag2_2023_02_10-07_58_20", "rosbag2_2023_02_10-07_58_47", "rosbag2_2023_02_10-07_58_54", "rosbag2_2023_02_11-09_12_52", "rosbag2_2023_02_11-09_13_01", "rosbag2_2023_02_11-09_13_09", ],
        GESTURE[3]: ["rosbag2_2023_02_10-08_03_45", "rosbag2_2023_02_10-08_03_55", "rosbag2_2023_02_10-08_04_04", "rosbag2_2023_02_11-09_19_44", "rosbag2_2023_02_11-09_19_52", "rosbag2_2023_02_11-09_20_02", ],
        GESTURE[4]: ["rosbag2_2023_02_10-08_09_42", "rosbag2_2023_02_10-08_10_25", "rosbag2_2023_02_10-08_10_34", "rosbag2_2023_02_11-09_23_36", "rosbag2_2023_02_11-09_23_45", "rosbag2_2023_02_11-09_23_54", ],
        GESTURE[5]: ["rosbag2_2023_02_10-08_15_11", "rosbag2_2023_02_10-08_15_22", "rosbag2_2023_02_10-08_15_35", "rosbag2_2023_02_11-09_33_23", "rosbag2_2023_02_11-09_34_03", "rosbag2_2023_02_11-09_34_15", ],
        GESTURE[6]: ["rosbag2_2023_02_10-08_20_28", "rosbag2_2023_02_10-08_20_38", "rosbag2_2023_02_10-08_20_56", "rosbag2_2023_02_11-10_24_48", "rosbag2_2023_02_11-10_24_54", "rosbag2_2023_02_11-10_25_01", ],
    }
    
    for gesture in GESTURE:
        fig, axs = plt.subplots(3, 6, figsize=(20, 10))
        for i, filename in enumerate(dataset[gesture]):
            raw_data = pd.read_csv(f'./rosbag/data/data/{filename}_data.csv')
            label = pd.read_csv(f'./rosbag/data/label/{filename}_label.csv').to_numpy()[0][0]
            raw_data = transform_to_relative_time(raw_data)

            acc_axes = axs[:, :3].ravel()
            vel_axes = axs[:, 3:].ravel()
            acc_data = [raw_data[f'Imu{i}_linear_accleration_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
            vel_data = [raw_data[f'Imu{i}_angular_velocity_{xyz}'] for i in range(3) for xyz in ['x', 'y', 'z']]
            acc_titles = [f'Imu{i}_acc_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]
            vel_titles = [f'Imu{i}_vel_{xyz}' for i in range(3) for xyz in ['x', 'y', 'z']]
            
            for ax, data, title in zip(acc_axes, acc_data, acc_titles):
                ax.plot(raw_data['timestamp'], data)
                ax.set_title(title)
                ax.set_ylim([-1.5 * 9.8, 1.5 * 9.8])
                
            for ax, data, title in zip(vel_axes, vel_data, vel_titles):
                ax.plot(raw_data['timestamp'], data)
                ax.set_title(title)
                ax.set_ylim([-5, 5])

            fig.suptitle(f'{GESTURE[label]}')
    plt.show()

def main(args=None):
    analyze_gesture()

if __name__ == '__main__':
    main()
