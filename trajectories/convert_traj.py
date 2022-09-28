import numpy as np
import sys
import os
import json
import argparse
import itertools
import copy

def main():

    parser = argparse.ArgumentParser(description='Convert a folder of csv files and adjust for actual flight')
    parser.add_argument('folder', help="Folder of x,y,z csv files, one csv for each trajectory")
    parser.add_argument('--velocity', '-v', type=float, help='Velocity of vehicle', default=1.0)
    parser.add_argument('--scale_factor', '-s', type=float, help='Scale all points equallty', default=1.0)
    parser.add_argument('--scale_x', '-sx', type=float, help='Scale only X points', default=1.0)
    parser.add_argument('--scale_y', '-sy', type=float, help='Scale only Y points', default=1.0)
    parser.add_argument('--scale_z', '-sz', type=float, help='Scale only Z points', default=1.0)
    parser.add_argument('--offset_z', '-z', type=float, help='Translate model upwards', default=0.5)
    parser.add_argument('--swap_xy', help="Swap X and Y axis",action='store_true')
    parser.add_argument('--output_file', '-o', help="Prefix for the output file", default="output")
    parser.add_argument('--translate_centroids', '-t', help="Translate X,Y centroids for each vehicle", nargs="+")
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
        print(f)
        traj = np.genfromtxt(os.path.join(full_folder, f), delimiter=',')

        if args.swap_xy:
            print("Swapping x and y")
            traj[:, [0, 1]] = traj[:, [1, 0]]

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

    # Also output a second file with moved centroids for each vehicle
    if args.translate_centroids is not None:

        if len(args.translate_centroids) != len(output) * 2:
            print(f"An x, y pair is needed for each vehicle, not enough provided: {args.translate_centroids}")
            exit()

        centroids = np.reshape([float(x) for x in args.translate_centroids], (-1, 2))
        print(centroids)

        trans_comb_folder = os.path.join(full_folder, f"translated_combinations_vel{velocity}_{'_'.join(args.translate_centroids)}")
        try:
            os.mkdir(trans_comb_folder)
        except OSError as error:
            print(error)
        for idxs in itertools.permutations(range(len(output))):
            new_output = copy.deepcopy(output)
            for i, j in enumerate(idxs):
                noj = copy.deepcopy(output[j])
                data = np.array(noj['data'])
                data[:, [1,2]] += centroids[i]
                new_output[i]["data"] = data.tolist()

            output_filename = f"{'_'.join([f't{i}' for i in idxs])}.json"
            with open(os.path.join(trans_comb_folder, output_filename), 'w') as f:
                json.dump(new_output, f)
            print(f"saved {output_filename}")


if __name__=="__main__":
    main()