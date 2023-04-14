import json

def load_data(filepath='/home/ubuntu/FYP-ROS/rosbag/metadata.json', ignore_dates=[], ignore_classes=[]):
    '''
    load data from metadata.json
    :param filepath: path to metadata.json
    :param ignore_dates: list of dates to ignore
    :param ignore_classes: list of classes to ignore
    :return: dataset, count

    :usage: 
    metadata, count = load_data()
    for gesture_type in tqdm.tqdm(metadata):
        for data_id in metadata[gesture_type]:
    '''
    dataset = {}
    count = 0
    with open(filepath, 'r') as f:
        metadata = json.load(f)
    for gesture in metadata:
        if gesture in ignore_classes:
            continue
        dataset[gesture] = []
        for entry in metadata[gesture]:
            if entry['date'] in ignore_dates:
                continue
            dataset[gesture].append(entry['data_id'])
            count += 1
    return dataset, count