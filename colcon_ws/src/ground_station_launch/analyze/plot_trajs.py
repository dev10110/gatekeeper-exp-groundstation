import numpy as np
import matplotlib.pyplot as plt
import pickle


def get_index_upto_time(msgs, t):

    for (i, m) in enumerate(msgs):
        # print(m[0])
        # print(t)
        # print(m[0] - t)
        # print()
        if m[0] > t:
            
            # print("returning")
            return i-1
    
    return len(msgs)-1

# maps = pickle.load("maps.pkl")
# nom_trajs = pickle.load("nom_trajs.pkl")
# com_trajs = pickle.load("com_trajs.pkl")
with open("actual_traj.pkl", "rb") as f:
  actual_traj = pickle.load(f)

with open("nom_trajs.pkl", "rb") as f:
  nom_trajs = pickle.load(f)

with open("com_trajs.pkl", "rb") as f:
  com_trajs = pickle.load(f)

with open("maps.pkl", "rb") as f:
  maps = pickle.load(f)


ts = [p[0]  for p in actual_traj]

# determine takeoff time 
takeoff_t = [p for p in actual_traj if p[3] > 0.2][0][0]

# determine when we are in the air 
is_in_air = [p for p in actual_traj if p[3] > 0.8]

# get five critical times
crit_times = np.linspace(takeoff_t, is_in_air[-1][0], num=5)


# plot the maps 
fig, axs = plt.subplots(1, 5)
for (i, ax) in enumerate(axs):

    ind = get_index_upto_time(maps, crit_times[i])

    m = maps[ind]

    ax.pcolormesh(m[1], m[2], -m[3].toarray(), cmap='Reds_r')

# plot the actual trajectories
for (i, ax) in enumerate(axs):

    ind = get_index_upto_time(actual_traj, crit_times[i])

    actual_x = [p[1] for p in actual_traj[:ind]]
    actual_y = [p[2] for p in actual_traj[:ind]]

    ax.plot(actual_x, actual_y, color="gray")

    ax.set_xlim([-2.4, 2])
    ax.set_ylim([-3, 1])
    ax.set_aspect('equal')
    ax.set_title(f"{(crit_times[i] - crit_times[0])* 1e-9:.1f}")

# plot the nominal trajectories
for (i, ax) in enumerate(axs):

    ind = get_index_upto_time(nom_trajs, crit_times[i])

    nom_traj = nom_trajs[ind]

    ax.plot(nom_traj[1], nom_traj[2], '--', color="blue")

# plot the committed trajectories
for (i, ax) in enumerate(axs):

    ind = get_index_upto_time(com_trajs, crit_times[i])

    com_traj = com_trajs[ind]

    ax.plot(com_traj[1], com_traj[2], linewidth=3, color="green")




plt.show()
