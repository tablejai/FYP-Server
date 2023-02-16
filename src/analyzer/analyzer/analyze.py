import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

gesture=["STATIC", "SLIDE_UP", "SLIDE_DOWN", "SLIDE_LEFT", "SLIDE_RIGHT", "ZOOM_IN", "ZOOM_OUT", "HIGHLIGHT", "ON_YES", "OFF_NO", "END"]

def transform_relative_time(data):
    data['timestamp'] = data['timestamp'] - data['timestamp'][0]
    return data

def main(args=None):
    dataset = [
        'rosbag2_2023_02_10-06_45_38', 
        'rosbag2_2023_02_10-07_49_15',
        'rosbag2_2023_02_10-07_58_20',
        'rosbag2_2023_02_10-08_03_45',
        'rosbag2_2023_02_10-08_09_42',
        'rosbag2_2023_02_10-08_15_11',
        'rosbag2_2023_02_10-08_20_28'
    ]

    for data in dataset:
        raw_data = pd.read_csv(f'./rosbag/data/data/{data}_data.csv')
        label = pd.read_csv(f'./rosbag/data/label/{data}_label.csv').to_numpy()[0][0]
        raw_data = transform_relative_time(raw_data)

        fig, axs = plt.subplots(3, 3, figsize=(15, 10))

        axs[0, 0].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_x'], label='ax0')
        axs[0, 0].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_y'], label='ay0')
        axs[0, 0].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_z'], label='az0')
        axs[0, 1].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_x'], label='wx0')
        axs[0, 1].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_y'], label='wy0')
        axs[0, 1].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_z'], label='wz0')
        axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_orientation_x'], label='ox0')
        axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_orientation_y'], label='oy0')
        axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_orientation_z'], label='oz0')
        axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_orientation_w'], label='ow0')

        axs[1, 0].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_x'], label='ax1')
        axs[1, 0].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_y'], label='ay1')
        axs[1, 0].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_z'], label='az1')
        axs[1, 1].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_x'], label='wx1')
        axs[1, 1].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_y'], label='wy1')
        axs[1, 1].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_z'], label='wz1')
        axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_orientation_x'], label='ox1')
        axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_orientation_y'], label='oy1')
        axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_orientation_z'], label='oz1')
        axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_orientation_w'], label='ow1')

        axs[2, 0].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_x'], label='ax2')
        axs[2, 0].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_y'], label='ay2')
        axs[2, 0].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_z'], label='az2')
        axs[2, 1].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_x'], label='wx2')
        axs[2, 1].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_y'], label='wy2')
        axs[2, 1].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_z'], label='wz2')
        axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_orientation_x'], label='ox2')
        axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_orientation_y'], label='oy2')
        axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_orientation_z'], label='oz2')
        axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_orientation_w'], label='ow2')

        fig.suptitle(f'{gesture[label]}')

        # for ax in axs.flat:
        #     ax.legend(loc='upper left')
        
        # plt.savefig(f'{gesture[label]}.png')

    plt.show()

if __name__ == '__main__':
    main()
