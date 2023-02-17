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

def plot_all_gesture():
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
        raw_data = transform_to_relative_time(raw_data)

        fig, axs = plt.subplots(3, 3, figsize=(15, 10))

        axs[0, 0].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_x'], label='ax0')
        axs[0, 0].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_y'], label='ay0')
        axs[0, 0].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_z'], label='az0')
        axs[0, 0].set_ylim([-2*9.8, 2*9.8])
        axs[0, 1].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_x'], label='wx0')
        axs[0, 1].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_y'], label='wy0')
        axs[0, 1].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_z'], label='wz0')
        axs[0, 1].set_ylim([-5, 5])
        axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_orientation_x'], label='ox0')
        axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_orientation_y'], label='oy0')
        axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_orientation_z'], label='oz0')
        axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_orientation_w'], label='ow0')
        axs[0, 2].set_ylim([-1, 1])

        axs[1, 0].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_x'], label='ax1')
        axs[1, 0].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_y'], label='ay1')
        axs[1, 0].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_z'], label='az1')
        axs[1, 0].set_ylim([-2*9.8, 2*9.8])
        axs[1, 1].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_x'], label='wx1')
        axs[1, 1].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_y'], label='wy1')
        axs[1, 1].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_z'], label='wz1')
        axs[1, 1].set_ylim([-5, 5])
        axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_orientation_x'], label='ox1')
        axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_orientation_y'], label='oy1')
        axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_orientation_z'], label='oz1')
        axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_orientation_w'], label='ow1')
        axs[1, 2].set_ylim([-1, 1])

        axs[2, 0].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_x'], label='ax2')
        axs[2, 0].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_y'], label='ay2')
        axs[2, 0].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_z'], label='az2')
        axs[2, 0].set_ylim([-2*9.8, 2*9.8])
        axs[2, 1].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_x'], label='wx2')
        axs[2, 1].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_y'], label='wy2')
        axs[2, 1].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_z'], label='wz2')
        axs[2, 1].set_ylim([-5, 5])
        axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_orientation_x'], label='ox2')
        axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_orientation_y'], label='oy2')
        axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_orientation_z'], label='oz2')
        axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_orientation_w'], label='ow2')
        axs[2, 2].set_ylim([-1, 1])

        fig.suptitle(f'{GESTURE[label]}')

        for ax in axs.flat:
            ax.legend(loc='upper right')
            ax.set_xlim([0, 10])
            
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
        dataset_ = dataset[gesture]
        fig, axs = plt.subplots(3, 6, figsize=(20, 10))
        for data in dataset_:
            raw_data = pd.read_csv(f'./rosbag/data/data/{data}_data.csv')
            label = pd.read_csv(f'./rosbag/data/label/{data}_label.csv').to_numpy()[0][0]
            raw_data = transform_to_relative_time(raw_data)


            axs[0, 0].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_x'])
            axs[0, 0].set_title(f'Imu0_acc_x')
            axs[0, 1].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_y'])
            axs[0, 1].set_title(f'Imu0_acc_y')
            axs[0, 2].plot(raw_data['timestamp'], raw_data['Imu0_linear_accleration_z'])
            axs[0, 2].set_title(f'Imu0_acc_z')

            axs[1, 0].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_x'])
            axs[1, 0].set_title(f'Imu1_acc_x')
            axs[1, 1].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_y'])
            axs[1, 1].set_title(f'Imu1_acc_y')
            axs[1, 2].plot(raw_data['timestamp'], raw_data['Imu1_linear_accleration_z'])
            axs[1, 2].set_title(f'Imu1_acc_z')

            axs[2, 0].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_x'])
            axs[2, 0].set_title(f'Imu2_acc_x')
            axs[2, 1].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_y'])
            axs[2, 1].set_title(f'Imu2_acc_y')
            axs[2, 2].plot(raw_data['timestamp'], raw_data['Imu2_linear_accleration_z'])
            axs[2, 2].set_title(f'Imu2_acc_z')
            
            for i in range(3):
                for j in range(3):
                    axs[j, i].set_ylim([-1.5*9.8, 1.5*9.8])

            axs[0, 3].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_x'])
            axs[0, 3].set_title(f'Imu0_vel_x')
            axs[0, 4].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_y'])
            axs[0, 4].set_title(f'Imu0_vel_y')
            axs[0, 5].plot(raw_data['timestamp'], raw_data['Imu0_angular_velocity_z'])
            axs[0, 5].set_title(f'Imu0_vel_z')
            
            axs[1, 3].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_x'])
            axs[1, 3].set_title(f'Imu1_vel_x')
            axs[1, 4].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_y'])
            axs[1, 4].set_title(f'Imu1_vel_y')
            axs[1, 5].plot(raw_data['timestamp'], raw_data['Imu1_angular_velocity_z'])
            axs[1, 5].set_title(f'Imu1_vel_z')

            axs[2, 3].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_x'])
            axs[2, 3].set_title(f'Imu2_vel_x')
            axs[2, 4].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_y'])
            axs[2, 4].set_title(f'Imu2_vel_y')
            axs[2, 5].plot(raw_data['timestamp'], raw_data['Imu2_angular_velocity_z'])
            axs[2, 5].set_title(f'Imu2_vel_z')

            for i in range(3):
                for j in range(3):
                    axs[j, i+3].set_ylim([-5, 5])
            
        fig.suptitle(f'{GESTURE[label]}')

def main(args=None):
    # plot_all_gesture()
    analyze_gesture()
    plt.show()

if __name__ == '__main__':
    main()
