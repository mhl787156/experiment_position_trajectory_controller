import numpy as np
import sys
import os
import json
import argparse

def main():

    parser = argparse.ArgumentParser(description='Convert a folder of csv files and adjust for actual flight')
    parser.add_argument('folder', help="Folder of x,y,z csv files, one csv for each trajectory")
    parser.add_argument('--velocity', '-v', type=float, help='Velocity of vehicle', default=1.0)
    parser.add_argument('--scale_factor', '-s', type=float, help='Scale all points equallty', default=1.0)
    parser.add_argument('--scale_x', '-sx', type=float, help='Scale only X points', default=1.0)
    parser.add_argument('--scale_y', '-sy', type=float, help='Scale only Y points', default=1.0)
    parser.add_argument('--scale_z', '-sz', type=float, help='Scale only Z points', default=1.0)
    parser.add_argument('--offset_z', '-z', type=float, help='Translate model upwards', default=0.5)
    parser.add_argument('--output_file', '-o', help="Prefix for the output file", default="output")
    args = parser.parse_args()
    print(args)

    velocity = args.velocity

    current_dir = os.getcwd()
    full_folder = os.path.join(current_dir, args.folder)

    columns = ["time", "x", "y", "z"]
    output = {}

    for idx, f in enumerate(os.listdir(full_folder)):
        if 'csv' not in f:
            continue
        traj = np.genfromtxt(os.path.join(full_folder, f), delimiter=',')
        traj *= args.scale_factor
        traj[:, 0] *= args.scale_x
        traj[:, 1] *= args.scale_y
        traj[:, 2] *= args.scale_z
        traj[:, 2] += args.offset_z

        # Calculate timings
        time = 0
        times = [0]
        for (t1, t2) in zip(traj[:len(traj)-1, :], traj[1:, :]):
            dist = np.linalg.norm(t2-t1)
            time += dist/velocity
            times.append(time)
        traj = np.insert(traj, 0, times, axis=1)

        output[idx] = {}
        output[idx]["columns"] = columns
        output[idx]["data"] =  traj.tolist()

    output_filename = f"{args.output_file}_vel{velocity}.json"
    with open(os.path.join(full_folder, output_filename), 'w') as f:
        json.dump(output, f)

if __name__=="__main__":
    main()