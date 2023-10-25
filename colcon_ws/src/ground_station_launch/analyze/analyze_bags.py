import argparse
import nml_bag
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import re
import csv
from scipy.sparse import csr_matrix
import pickle

def to_ms(record):
    return 1e-6 * (1e9 * record["sec"] + record["nanosec"])


def list_topics(bagfile):

    reader = nml_bag.Reader(bagfile)

    return reader.topics

def collect_by_topic(bagfile, topic):

    reader = nml_bag.Reader(bagfile, topics = [topic])

    ts = []
    msgs = []

    for msg in reader:

        ts.append(msg["time_ns"])
        msgs.append(msg)
        return (ts, msgs)

def process_occ_grid(bagfile):

    reader = nml_bag.Reader(bagfile, topics = ["/global_map"])

    maps = []

    for msg in reader:

        # load data
        t = msg["info"]["map_load_time"]["sec"] * 1e9 + msg["info"]["map_load_time"]["nanosec"]
        print(t)
        w = msg["info"]["width"]
        h = msg["info"]["height"]
        origin_x = msg["info"]["origin"]["position"]["x"]
        origin_y = msg["info"]["origin"]["position"]["y"]
        res = msg["info"]["resolution"]
        data = np.reshape(np.array(msg["data"]), [h, w]) 

        # convert to sparse
        data = csr_matrix(data)

        # create axes
        xs = np.arange(0, w) * res + origin_x
        ys = np.arange(0, h) * res + origin_y
        X, Y = np.meshgrid(xs, ys)

        # save to data structure
        maps.append(
                (t, X, Y, data)
                )
        
        # try plotting
        # plt.figure()
        # plt.pcolormesh(X, Y, data.toarray())
        # plt.show()
        # return
    
    return maps


def process_trajs(bagfile, topic):

    reader = nml_bag.Reader(bagfile, topics=[topic])

    traj = []

    for msg in reader:
        t = msg["header"]["stamp"]["sec"] * 1e9 + msg["header"]["stamp"]["nanosec"]
        print(t)

        xs = [p["position"]["x"] for p in msg["poses"]]
        ys = [p["position"]["y"] for p in msg["poses"]]
        zs = [p["position"]["z"] for p in msg["poses"]]

        traj.append((t, xs, ys, zs))

    return traj

def process_actual_traj(bagfile):

    reader = nml_bag.Reader(bagfile, topics=["/tf"])

    path = []

    for msg in reader:
        transforms = msg["transforms"]
        for trans in transforms:
            if trans["child_frame_id"] == "vicon/px4_1/px4_1":
                t = trans["header"]["stamp"]["sec"] * 1e9 + trans["header"]["stamp"]["nanosec"]
                print(t)
                x = trans["transform"]["translation"]["x"]
                y = trans["transform"]["translation"]["y"]
                z = trans["transform"]["translation"]["z"]
                path.append( (t, x, y, z))

    print(path)
    return path






if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("bagfile")

    args = parser.parse_args()

    print(f"processing {args.bagfile}")

    topics = list_topics(args.bagfile)

    print(f"topic are: {topics}")

    # read all the maps
    if False: # already have run this
      maps = process_occ_grid(args.bagfile)
      maps_file = "./maps.pkl"
      with open(maps_file, "wb") as outp:
          pickle.dump(maps, outp)

    # read all the trajectories
    if False:
        com_trajs = process_trajs(args.bagfile, "/committed_traj/viz")
        com_trajs_file = "./com_trajs.pkl"
        with open(com_trajs_file, "wb") as outp:
            pickle.dump(com_trajs, outp)

        nom_trajs = process_trajs(args.bagfile, "/nominal_traj/viz")
        nom_trajs_file = "./nom_trajs.pkl"
        with open(nom_trajs_file, "wb") as outp:
            pickle.dump(nom_trajs, outp)

    # read all the tfs
    if False:

        actual_traj = process_actual_traj(args.bagfile)
        actual_traj_file = "./actual_traj.pkl"
        with open(actual_traj_file, "wb") as outp:
            pickle.dump(actual_traj, outp)





