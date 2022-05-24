import numpy as np
import sys
import os
import json

def main():
    if len(sys.argv) < 3:
        print("Please give folder of x,y,z csv files to convert, followed by a velocity")
        exit()

    folder = sys.argv[1]
    velocity = float(sys.argv[2])

    scale_factor = 1.0
    z_offset = 0.5

    current_dir = os.getcwd()
    full_folder = os.path.join(current_dir, folder)

    columns = ["time", "x", "y", "z"]
    output = {}

    for idx, f in enumerate(os.listdir(full_folder)):
        traj = np.genfromtxt(os.path.join(full_folder, f), delimiter=',')
        traj *= scale_factor
        traj[:, 2] += z_offset

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

    output_filename = f"output_vel{velocity}.json"
    with open(os.path.join(full_folder, output_filename), 'w') as f:
        json.dump(output, f)

if __name__=="__main__":
    main()