#!/usr/bin/env/python3

"""
Script that generates random data for testing purposes.

The script will read the landmarks YAML file and the poses from poses.csv
and generate random measurements from each pose.
Each list of measurements will be created in a CSV file with the timestamp in the file name.

The measurements will be generated based on a maximum detection distance
and a normal distribution for the detection error. Those parameters can be modified
"""

import csv
import os

import numpy as np
import yaml


LANDMARKS_FILE = 'test_landmarks.yaml'
MEASUREMENTS_DIR = 'measurements'
POSES_FILE = 'measurements/poses.csv'
MAX_RANGE = 20.0
ERROR_STD = 0.1


def load_landmarks():
    if not os.path.isfile(LANDMARKS_FILE):
        print(F'ERROR: Landmarks file "{LANDMARKS_FILE}" does not exist')
        raise FileNotFoundError
    with open(LANDMARKS_FILE, 'r') as landmarks_file:
        landmarks_doc = yaml.safe_load(landmarks_file)
    landmarks = []
    ids = []
    for l_doc in landmarks_doc['landmarks']:
        ids.append(l_doc['id'])
        landmarks.append(l_doc['coords'])
    return np.array(landmarks), np.array(ids)


def load_poses():
    if not os.path.isdir(MEASUREMENTS_DIR):
        print(F'ERROR: Measurements directory "{MEASUREMENTS_DIR}" does not exist')
        raise FileNotFoundError
    if not os.path.isfile(POSES_FILE):
        print(F'ERROR: Poses file "{POSES_FILE}" does not exist')
        raise FileNotFoundError
    timestamps = []
    poses = []
    with open(POSES_FILE) as poses_csv:
        poses_reader = csv.reader(poses_csv, delimiter=';')
        for row in poses_reader:
            timestamps.append(int(row[0]))
            pose = [
                float(row[1]),
                float(row[2]),
                float(row[3]),
            ]
            poses.append(pose)
    return np.array(timestamps), np.array(poses)


def detect_landmarks(landmarks, ids, pose):
    rot_angle = - pose[2]
    rot_mat = np.array([
        [np.cos(rot_angle), - np.sin(rot_angle)],
        [np.sin(rot_angle), np.cos(rot_angle)]
    ])
    diff = (landmarks - pose[:2]).T
    relative_landmarks = rot_mat.dot(diff).T
    # Remove landmarks out of detection range
    distances = np.hypot(relative_landmarks[:, 0], relative_landmarks[:, 1])
    relative_landmarks = relative_landmarks[distances <= MAX_RANGE]
    ids = ids[distances <= MAX_RANGE]
    # Add noise
    relative_landmarks += np.random.normal(0, ERROR_STD, relative_landmarks.shape)
    return relative_landmarks, ids


def save_measurements(timestamp, measurements, ids):
    filename = F'measurements_{timestamp}.csv'
    measurement_file = os.path.join(MEASUREMENTS_DIR, filename)
    covariance = [ERROR_STD**2, 0.0, 0.0, ERROR_STD**2]
    data_lines = []
    # Iterate to apply custom float formatting
    for i, m in zip(ids, measurements):
        line = [i, F'{m[0]:.4f}', F'{m[1]:.4f}']
        line += [F'{x:.4f}' for x in covariance]
        data_lines.append(line)
    with open(measurement_file, 'w') as out_csv:
        m_writer = csv.writer(out_csv, delimiter=';')
        m_writer.writerows(data_lines)


def main():
    landmarks, ids = load_landmarks()
    print(F'Landmarks:\n{landmarks}')
    print(F'Landmark IDs:\n{ids}')
    timestamps, poses = load_poses()
    print(F'Timestamps:\n{timestamps}')
    print(F'Poses:\n{poses}')
    # To test detection function
    # detect_landmarks(
    #     np.array([[5.0, 5.0], [2.0, 4.0]]),
    #     np.array([4, 8]),
    #     np.array([0.0, 0.0, np.pi/2])
    # )
    for t, p in zip(timestamps, poses):
        print('-----------------------------')
        print(F'Pose: {t} -> {p}')
        measurements, detected_ids = detect_landmarks(landmarks, ids, p)
        print('Measurements [ID -> Detection]:')
        for m, i in zip(measurements, detected_ids):
            print(F'{i:2d} -> {m}')
        save_measurements(t, measurements, detected_ids)


if __name__ == '__main__':
    main()
