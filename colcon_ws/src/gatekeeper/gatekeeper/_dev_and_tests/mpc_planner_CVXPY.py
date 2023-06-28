
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

class MPCPlanner: 

    def __init__(self,
            N=20,  # planning horizon
            DT=0.1, # in seconds
            max_accel = 3.0 # m/s^2
            ):

        self.N = N
        self.DT = DT
        self.max_accel = max_accel

        A = np.array([
            [1, DT],
            [0, 1.0]])

        B = np.array([[0.5*DT*DT],[DT]]) 

        I3 = np.eye(3)
        self.dyn_A = np.kron(I3, A)
        self.dyn_B = np.kron(I3, B)


        # initial goal and x0
        self.goal = np.zeros(6)
        self.x0 = np.zeros(6)

        # initialize sfc_A, sfc_b
        # constraint is of the form sfc_A @ x[pos_inds] <= sfc_b 
        self.sfc_A = np.array([[1.0, 0, 0]])
        self.sfc_b = np.array([100.0]) # default initializer just so there is something to work with

    def set_start_state(self, state):
        self.x0 = state
    
    def set_target_state(self, target_state):
        self.goal = target_state

    def set_target_location(self, target):
        for i in range(3):
            self.goal[2*i] = target[i]
            self.goal[2*i+1] = 0.0

    def set_safe_polyhedron(self, A, b):

        self.sfc_A = A
        self.sfc_b = b

    def solve(self):
       
       ### Construct the variables
       xs = [cp.Variable(6) for i in range(self.N)]
       us = [cp.Variable(3) for i in range(self.N)]

       ### construct the cost function
       Q = np.eye(6)
       obj = sum( cp.quad_form(x - self.goal, Q) for x  in xs)
       print("Goal: ", self.goal)

       ### construct the constraints
       cons = []
       
       # accel limits
       for u in us:
           cons.append( cp.norm(u, 'inf') <= self.max_accel)

       # dynamics
       for i in range(self.N-1):
           cons.append( xs[i+1] == self.dyn_A @ xs[i] + self.dyn_B @ us[i] )

       # initial condition
       cons.append(xs[0] == self.x0)
       
       # end at a stop
       for i in range(3):
           cons.append(xs[-1][2*i + 1] == 0.0)
       cons.append(us[-1] == 0.0)

       # sfc
       pos_inds = [0, 2, 4]
       for x in xs:
           cons.append( self.sfc_A @ x[pos_inds] <= self.sfc_b )

       ### Construct the problem
       prob = cp.Problem(cp.Minimize(obj), cons)
       result = prob.solve(solver=cp.OSQP)

       print(prob.status)

       self.sol_x = [x.value for x in xs]
       self.sol_u = [u.value for u in us]

       return self.sol_x, self.sol_u

    def plot_solution(self):

        xs = [x[0] for x in self.sol_x]
        ys = [x[2] for x in self.sol_x]
        zs = [x[4] for x in self.sol_x]

        fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
        ax.stem(xs, ys, zs)

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")

        plt.show()

        return


