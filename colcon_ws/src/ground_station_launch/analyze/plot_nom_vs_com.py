import numpy as np
import matplotlib.pyplot as plt
import pickle


def get_index_upto_time(msgs, t):

    for (i, m) in enumerate(msgs):
        if m[0] > t:
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


ts = [p[0]  for p in actual_traj]

# determine takeoff time 
takeoff_t = [p for p in actual_traj if p[3] > 0.2][0][0]

# determine when we are in the air 
is_in_air = [p for p in actual_traj if p[3] > 0.8]

end_t = is_in_air[-1][0]

start_ind_nom = get_index_upto_time(nom_trajs, takeoff_t)
start_ind_com = get_index_upto_time(com_trajs, takeoff_t)
end_ind_nom = get_index_upto_time(nom_trajs, end_t)
end_ind_com = get_index_upto_time(com_trajs, end_t)


pub_t_nom = np.array([p[0] for p in nom_trajs[start_ind_nom:end_ind_nom]])
pub_t_com = np.array([p[0] for p in com_trajs[start_ind_com:end_ind_com]])

plt.figure()
plt.plot((pub_t_nom - takeoff_t)*1e-9, 1*np.ones(len(pub_t_nom)), 'x')
plt.plot((pub_t_com - takeoff_t)*1e-9, 2*np.ones(len(pub_t_com)), '.')

plt.gca().set_xlim([-5, 80])
plt.xlabel("Time [s]")


plt.show()
