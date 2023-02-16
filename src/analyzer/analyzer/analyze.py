import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main(args=None):
    data = pd.read_csv('./rosbag/data/data/rosbag2_2023_02_10-06_45_38_data.csv')
    print(data)

    plt.plot(data['timestamp'], data['Imu0_linear_accleration_x'], label='Imu0_linear_accleration_x')
    plt.plot(data['timestamp'], data['Imu0_linear_accleration_y'], label='Imu0_linear_accleration_y')
    plt.plot(data['timestamp'], data['Imu0_linear_accleration_z'], label='Imu0_linear_accleration_z')
    plt.legend()
    plt.show()
    
if __name__ == '__main__':
    main()
