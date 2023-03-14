GESTURES = [
    "STATIC",
    "SLIDE_UP",
    "SLIDE_DOWN",
    "SLIDE_LEFT",
    "SLIDE_RIGHT",
    "RELEASE",
    "GRASP",
    "NONE",
]

# Spectral / 10 color palette
colors_palette = {
    "STATIC": "#d53e4f",
    "SLIDE_UP": "#f46d43",
    "SLIDE_DOWN": "#abdda4",
    "SLIDE_LEFT": "#3288bd",
    "SLIDE_RIGHT": "#5e4fa2",
    "RELEASE": "#fdae61",
    "GRASP": "#66c2a5",
    "NONE": "#3288bd",
}

def load_data(filepath='/home/ubuntu/FYP-ROS/rosbag/bag/info.txt'):
    dataset = {}
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            for gesture in GESTURES:
                if line.startswith(f'{gesture}:'):
                    key = gesture
                    dataset[key] = []
                    break
            else:
                if line != '':
                    dataset[key].append(line.split()[-1])
    return dataset