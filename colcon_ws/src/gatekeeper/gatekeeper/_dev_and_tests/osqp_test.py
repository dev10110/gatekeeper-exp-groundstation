import osqp
import numpy as np
from scipy import sparse
import time
import cvxpy as cp


def solve_osqp():

    # Define problem data
    P = sparse.csc_matrix([[4, 1], [1, 2.0]])
    q = np.array([1, 1.0])
    A = sparse.csc_matrix([[1, 1], [1, 0.0], [0, 1]])
    l = np.array([1, 0, 0.0])
    u = np.array([1, 0.7, 0.7])
    
    # Create an OSQP object
    prob = osqp.OSQP()
    
    # Setup workspace and change alpha parameter
    prob.setup(P, q, A, l, u, alpha=1.0, verbose=False)
    
    # Solve problem
    res = prob.solve()
    
    print(res.x)

def solve_cvxpy():

    # Define problem data
    P = np.array([[4, 1], [1, 2.0]])
    q = np.array([1, 1.0])
    A = np.array([[1, 1.0], [1, 0], [0, 1]])
    l = np.array([1, 0, 0.0])
    u = np.array([1, 0.7, 0.7])


    x = cp.Variable(2)

    obj = cp.Minimize(cp.quad_form(x, P) + q.T @  x)
    
    cons = [
            A @ x <= u,
            l <= A @ x
            ]
    
    prob = cp.Problem(obj, cons)

    # Solve problem
    res = prob.solve()
    
    # print(res.x)
    print(x.value)
    
if __name__=="__main__":
    print("***OSQP***")
    start = time.time()
    for i in range(10):
        solve_osqp()
    end = time.time()

    print((end-start)/10)
    
    print("***CVXPY***")
    start = time.time()
    for i in range(10):
        solve_cvxpy()
    end = time.time()

    print((end-start)/10)
