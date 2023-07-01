import time
import osqp
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pyplot as plt


## Based off the implementation suggested at
## https://osqp.org/docs/examples/mpc.html
## uses the same variable names as in that doc

class MPCPlanner: 

    def __init__(self,
            N=20,  # planning horizon
            DT=0.1, # in seconds
            max_accel = 3.0 # m/s^2
            ):

        ## save the params
        self.N = N
        self.DT = DT
        self.max_accel = max_accel

        ## dynamics
        self.create_dynamics_matrices()
        
        # initial goal and x0
        self.xr = np.ones(6)
        self.x0 = np.zeros(6)

        # initialize sfc_A, sfc_b
        # constraint is of the form sfc_A @ x[pos_inds] <= sfc_b 
        sfc_A = np.array([[1.0, 0, 0]])
        sfc_b = np.array([100]) # default initializer just so there is something to work with
        self.set_safe_polyhedron(sfc_A, sfc_b)


    def create_dynamics_matrices(self):
        A = np.array([
            [1, self.DT],
            [0, 1.0]])

        B = np.array([[0.5*self.DT**2],[self.DT]]) 

        I3 = np.eye(3)
        self.Ad = sparse.csc_matrix( np.kron(I3, A) )
        self.Bd = sparse.csc_matrix( np.kron(I3, B) )

        [nx, nu] = self.Bd.shape
        self.nx = nx # should be 6
        self.nu = nu # should be 3



    def set_state(self, state):
        self.x0 = state
    
    def set_target_state(self, target_state):
        self.xr = target_state

    def set_target_location(self, target):
        for i in range(3):
            self.xr[2*i] = target[i]
            self.xr[2*i+1] = 0.0

    def set_safe_polyhedron(self, A, b):

        N_sfc = A.shape[0]

        padded_A = np.insert(A, [1,2,3], np.zeros([N_sfc,3]), axis=1) # converts [a,b,c] to [a, 0, b, 0, c, 0]
        self.sfc_A = sparse.csc_matrix(padded_A)
        self.sfc_b = b


    def construct_objective(self):


        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        N = self.N
        nx = self.nx
        nu = self.nu

        Q = sparse.eye(nx)
        QN = N*Q
        R = 0.00001*sparse.eye(nu)

        P = sparse.block_diag(
                [sparse.kron(sparse.eye(self.N), Q), QN,
                       sparse.kron(sparse.eye(self.N), R)], format='csc')

        q = np.hstack([
            np.kron(np.ones(N), -Q@self.xr),
            -QN@self.xr,
            np.zeros(N*nu)])
        
        # print(P.shape)
        # print(np.shape(q))

        return P, q


    def construct_cons_dynamics(self):

        N = self.N
        nx = self.nx
        nu = self.nu

        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(N*nx)])
        ueq = leq

        return Aeq, leq, ueq


    def construct_cons_input(self):
        
        N = self.N
        nx = self.nx
        nu = self.nu

        umin = -self.max_accel;
        umax =  self.max_accel;

        Ax = sparse.csc_matrix(np.zeros([N*nu, (N+1)*nx]))
        Bu = sparse.eye(N*nu)

        Aineq = sparse.hstack([Ax, Bu])
        lineq = umin * np.ones(N*nu)
        uineq = umax * np.ones(N*nu)

        return Aineq, lineq, uineq


    def construct_cons_terminal(self):

        # forces the terminal speed to be 0
        
        N = self.N
        nx = self.nx
        nu = self.nu

        Aeq = sparse.hstack([
          sparse.csc_matrix(np.zeros([3, N*nx])),
          sparse.csc_matrix(np.array([ 
              [0, 1, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 1]])),
          sparse.csc_matrix(np.zeros([3, N*nu]))
          ])

        leq = np.zeros(3) 
        ueq = np.zeros(3)

        return Aeq, leq, ueq


    def construct_cons_sfc(self):

        N = self.N
        nx = self.nx
        nu = self.nu

        N_sfc = self.sfc_A.shape[0] # number of constraints in the polyhedron

        # diagonalize it
        Ax = sparse.kron(sparse.eye(N+1), sparse.csc_matrix(self.sfc_A))
        Bu = sparse.csc_matrix(np.zeros([(N+1)*N_sfc, N*nu]))

        Aineq = sparse.hstack([Ax, Bu])

        sfc_l = (-np.inf) * np.ones(N_sfc)
        
        lineq = np.kron(np.ones(N+1), sfc_l)
        uineq = np.kron(np.ones(N+1), self.sfc_b)

        return Aineq, lineq, uineq


    def construct_constraints(self):


        # dynamics + initial condition

        cons_dyn = self.construct_cons_dynamics() # also has the initial state constraint
        
        # input constraints
        cons_input = self.construct_cons_input()
        
        # must come to a stop
        cons_terminal = self.construct_cons_terminal()

        # todo: sfc
        cons_sfc = self.construct_cons_sfc()


        ## concatenate 

        cons = [cons_dyn,
                cons_input,
                cons_terminal,
                cons_sfc
                ]

        cons_A = sparse.vstack([c[0] for c in cons], format="csc")
        cons_l = np.hstack([c[1] for c in cons])
        cons_u = np.hstack([c[2] for c in cons])

        return cons_A, cons_l, cons_u


    def solve(self):

        ## Objective function
        P, q = self.construct_objective()

        ## constraints 
        A_cons, l_cons, u_cons = self.construct_constraints()
        # print(A_cons.shape, np.shape(l_cons), np.shape(u_cons))

        # Create an OSQP object
        prob = osqp.OSQP()
        
        # Setup workspace
        prob.setup(P, q, A_cons, l_cons, u_cons, verbose=False)

        # Solve
        res = prob.solve()

        # Check solve status
        if res.info.status_val != 1:
            print(res.info.status)
            return False

        # ok it solved optimally

        N = self.N
        nx = self.nx
        nu = self.nu

        # update the solution
        self.sol_x = [res.x[(nx*i):(nx*(i+1))] for i in range(N+1)]
        self.sol_u = [res.x[((N+1)*nx + nu*i):((N+1)*nx + nu*(i+1))] for i in range(N)]

        return True

    def print_solution(self):
        
        for i in range(self.N):
            print(f"{i}:: x={np.array2string(self.sol_x[i], precision=2, suppress_small=True)} u={np.array2string(self.sol_u[i], precision=2, suppress_small=True)}")

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

if __name__ == "__main__":

    mpc = MPCPlanner()


    start_time = time.time()
    mpc.set_target_location(np.ones(3))
    sol = mpc.solve()
    end_time = time.time()

    mpc.print_solution()

    print(f"Time: {end_time - start_time} seconds")
    mpc.plot_solution()

