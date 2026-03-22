"""
Mirrors choreo trajectories, which are labeled with the "mirrored_" prefix.
"""
import json
import os
import subprocess

field_height = 8.043
choreo_dir = "../../src/main/deploy/choreo"
# The traj files that shouldn't be mirrored.
ignore_mirrored = ["CloseFuelGrab.traj", "CloseFuelScore.traj", "SubstationGrab.traj", "SubstationScore.traj"]


def load_traj(file_path):
    with open(file_path, 'r') as f:
        traj = json.load(f)
    return traj


def mirror_waypoint(point):
    p_y_val = field_height - point["y"]["val"]
    p_y_exp = str(p_y_val) + " m"
    point["y"]["val"] = p_y_val
    point["y"]["exp"] = p_y_exp

    p_heading_val = -point["heading"]["val"]
    p_heading_exp = str(p_heading_val) + " rad"

    point["heading"]["val"] = p_heading_val
    point["heading"]["exp"] = p_heading_exp

    return point


def mirror_constraint(constraint):
    match constraint["data"]["type"]:
        case "KeepInRectangle":
            # KeepInRectangle constraints use the bottom left corner as the x and y coord
            # instead of the center, so we apply mirroring to the center instead.
            props = constraint["data"]["props"]
            center_y_coord = props["y"]["val"] + props["h"]["val"] / 2.0
            mirrored_center_y_coord = field_height - center_y_coord
            mirrored_y_coord = mirrored_center_y_coord - props["h"]["val"] / 2.0
            props["y"] = {
                "val": mirrored_y_coord,
                "exp": f"{mirrored_y_coord} m"
            }

        case "KeepInCircle" | "KeepOutCircle":
            props = constraint["data"]["props"]
            mirrored_y_coord = field_height - props["y"]["val"]
            props["y"] = {
                "val": mirrored_y_coord,
                "exp": f"{mirrored_y_coord} m"
            }

    return constraint


def main():
    choreo_files = os.listdir(choreo_dir)

    # Find only .traj files that do not contain the word "mirrored"
    traj_files = [file for file in choreo_files if file.endswith(".traj") and "mirrored" not in file]
    traj_files = [file for file in traj_files if file not in ignore_mirrored]
    for file in traj_files:
        traj = load_traj(choreo_dir + "\\" + file)
        waypoints = traj["params"]["waypoints"]
        constraints = traj["params"]["constraints"]

        for i, point in enumerate(waypoints):
            waypoints[i] = mirror_waypoint(point)

        for i, constraint in enumerate(constraints):
            constraints[i] = mirror_constraint(constraint)

        # Clear previously cached data
        traj["snapshot"]["waypoints"] = []
        traj["snapshot"]["constraints"] = []
        traj["trajectory"]["samples"] = []

        # save the mirrored traj
        with open(choreo_dir + "\\" + "mirrored_" + file, 'w') as f:
            json.dump(traj, f, indent=2)

    # Uses Choreo CLI to generate trajectories
    mirrored_traj_files = ["mirrored_" + file for file in traj_files]
    subprocess.run(
        f"%USERPROFILE%/AppData/Local/Choreo/choreo-cli.exe --chor {choreo_dir}/Autos.chor"
        f" --trajectory {",".join(mirrored_traj_files)} -g",
        shell=True
    )

if __name__ == '__main__':
    main()