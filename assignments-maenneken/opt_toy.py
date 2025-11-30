import cvxpy as cp
import numpy as np
from matplotlib import pyplot as plt

"""
start = (0.5, 2.5)
goal = (9.5, 0.5)

P0=start
P9=goal
Free space C1, C2, C3
3 Bezier curves a 4 Points
B1(t)=(1−t)^3 *P0+ 3(1−t)^2 *t*P1+3(1−t)*t^2*P2+t^3*P3 , t∈[0,1]
B2(t)=(1−t)^3 *P3+ 3(1−t)^2 *t*P4+3(1−t)*t^2*P5+t^3*P6 
B3(t)=(1−t)^3 *P6+ 3(1−t)^2 *t*P7+3(1−t)*t^2*P8+t^3*P9 

d_B1(1) = 3 * (P3 - P2)
d_B2(0) = 3 * (P4 - P3)
d_B2(1) = 3 * (P6 - P5)
d_B3(0) = 3 * (P7 - P6)

dd_B1(1) = -6 (P2-P1) + 6 (P3-P2) = 6(P1 -2P2 + P3)
dd_B2(0) = -6 (P4-P3) + 6 (P5-P4) = 6(P3 -2P4 + P5)
dd_B2(1) = -6 (P5-P4) + 6 (P6-P5) = 6(P4 -2P5 + P6)
dd_B3(0) = -6 (P7-P6) + 6 (P8-P7) = 6(P6 -2P7 + P8)

1. P0, P1, P2, P3 all in C1
2. P3, P4, P5, P6 all in C2
3. P6, P7, P8, P9 all in C3

P3 in C1 and C2
P6 in C2 and C3

C1 : 0 <= x <= 6; 2 <= y <= 3
C2 : 4 <= x <= 6; 0 <= y <= 5
C3 : 4 <= x <= 10; 0 <= y <= 1

Continues:
C1: d_B1(1) == d_B2(0), d_B2(1) == d_B3(0)
C2: dd_B1(1) == dd_B2(0), dd_B2(1) == dd_B3(0)

"""
def bezier(P,t):
    B = (1 - t)**3 * P[0] + 3*(1 - t)**2 * t* P[1] + 3*(1 - t) * t**2 * P[2] + t**3 * P[3]
    return B

def main():
    start = [0.5, 2.5]
    goal = [9.5, 0.5]

    P = cp.Variable((10, 2))

    constraints = [
        P[0] == start,
        P[9] == goal,
        # C1
        P[1, 0] >= 0, P[1, 0] <= 6,
        P[2, 0] >= 0, P[2, 0] <= 6,
        P[3, 0] >= 4, P[3, 0] <= 6,  # P3 in C1 and C2

        P[1, 1] >= 2, P[1, 1] <= 3,
        P[2, 1] >= 2, P[2, 1] <= 3,
        P[3, 1] >= 2, P[3, 1] <= 3,

        # C2
        P[4, 0] >= 4, P[4, 0] <= 6,
        P[5, 0] >= 4, P[5, 0] <= 6,
        P[6, 0] >= 4, P[6, 0] <= 6,  # P6 in C2 and C3

        P[4, 1] >= 0, P[4, 1] <= 5,
        P[5, 1] >= 0, P[5, 1] <= 5,
        P[6, 1] >= 0, P[6, 1] <= 1,

        # C3
        P[7, 0] >= 4, P[7, 0] <= 10,
        P[8, 0] >= 4, P[8, 0] <= 10,

        P[7, 1] >= 0, P[7, 1] <= 1,
        P[8, 1] >= 0, P[8, 1] <= 1,

        # C1 continues
        P[4] + P[2] == 2 * P[3],
        P[7] + P[5] == 2 * P[6],
        # C2 continues
        P[1] - 2 * P[2] + P[3] == P[3] - 2 * P[4] + P[5],
        P[4] - 2 * P[5] + P[6] == P[6] - 2 * P[7] + P[8]
    ]
    #minimize length of the path
    objective = cp.Minimize(cp.sum_squares(P[1:] - P[:-1]))
    prob = cp.Problem(objective, constraints)
    prob.solve()

    print("Control points:\n", P.value)

    t_vals = np.linspace(0, 1, 100)
    P_val = P.value

    seg1 = np.array([bezier(P_val[0:4], t) for t in t_vals])
    seg2 = np.array([bezier(P_val[3:7], t) for t in t_vals])
    seg3 = np.array([bezier(P_val[6:10], t) for t in t_vals])

    plt.figure()

    plt.plot(seg1[:, 0], seg1[:, 1], label="Curve 1")
    plt.plot(seg2[:, 0], seg2[:, 1], label="Curve 2")
    plt.plot(seg3[:, 0], seg3[:, 1], label="Curve 3")
    plt.plot(P_val[:, 0], P_val[:, 1], 'x', label="Control Points")

    # Obstacles / Regions
    plt.fill([0, 6, 6, 0], [2, 2, 3, 3], color='red', alpha=0.2, label='C1')
    plt.fill([4, 6, 6, 4], [0, 0, 5, 5], color='green', alpha=0.1, label='C2')
    plt.fill([4, 10, 10, 4], [0, 0, 1, 1], color='blue', alpha=0.1, label='C3')


    plt.xlim(0, 10)
    plt.ylim(0, 5)
    plt.title("Bézier Path")
    plt.legend()
    plt.grid(True)
    plt.savefig("opt_toy.pdf")


if __name__ == "__main__":
    main()
