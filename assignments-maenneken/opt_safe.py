import argparse
import yaml
import time
import cvxpy as cp
import numpy as np
from matplotlib import pyplot as plt
from numpy.ma.core import arctan2


def compute_hyperplane(obstacles, segment):
    """
    input: corners of obstacles and endpoints of a line segment
    return: w, b of SVM
    """
    w = cp.Variable(2)
    b = cp.Variable()

    constraints = []

    # Obstacle corners: w^T x - b >= 1
    for p in obstacles:
        constraints.append(w @ p - b >= 1)

    # Path points: w^T x - b <= -1
    for p in segment:
        constraints.append(w @ p - b <= -1)

    # Objective: minimize ||w||^2
    objective = cp.Minimize(cp.sum_squares(w))

    problem = cp.Problem(objective, constraints)
    problem.solve()

    return w.value, b.value

def bezier(P,t):
    B = (1 - t)**3 * P[0] + 3*(1 - t)**2 * t* P[1] + 3*(1 - t) * t**2 * P[2] + t**3 * P[3]
    return B

def dBezier(P,t):
    dB = 3 * ((1 - t)**2 * (P[1] - P[0]) + 2 * (1 - t) * t * (P[2] - P[1]) + t**2 * (P[3] - P[2]))
    return dB

def ddBezier(P,t):
    ddB = 6 * ((1 - t) * (P[2] - 2 * P[1] + P[0]) + t * (P[3] - 2 * P[2] + P[1]))
    return ddB


def compute_control_points(path,W,B,env_min,env_max):
    n_segments = len(path)-1

    P = cp.Variable((2*n_segments+len(path), 2))
    constraints = []

    for p in P:
        constraints+= [p[0]>=env_min[0], p[0]<=env_max[0], p[1]>=env_min[1], p[1]<=env_max[1]]

    for i in range(len(path)):
        #go through the path
        constraints.append(P[i*3]==path[i])
        if i == n_segments:
            break

        for w, b in zip(W[i],B[i]):
            #be inside the safe space
            constraints.append(w @ P[i*3 + 1] -b <= -1)
            constraints.append(w @ P[i*3 + 2] -b  <= -1)


    for i in range(1,len(path)-1):
        #C1
        constraints.append(P[3 * i +1] + P[3 * i - 1] == 2 * P[3 * i])
        #C2
        #constraints.append( P[3 * i - 2] - 2 * P[3 * i - 1] + P[3 * i] == P[3*i] - 2 * P[3 * i + 1] + P[3 * i + 2])

    # minimize length of the path
    objective = cp.Minimize(cp.sum_squares(P[1:] - P[:-1]))
    prob = cp.Problem(objective, constraints)
    prob.solve()

    return P.value

def hyperplane(w,b):
    #w @ x -b = 0 -> wx * x + wy * y = b
    wx, wy = w

    x = np.linspace(0, 10, 100)
    y = (b - w[0] * x) / w[1]
    return x,y


def inverseKinematics(q,L):
    #https: // robotacademy.net.au / lesson / inverse - kinematics -for -a - 2 - joint - robot - arm - using - geometry /
    #https://www.tu-chemnitz.de/informatik/KI/edu/robotik/ws2017/inv.kin.pdf
    x, y, theta = q
    L0, L1, L2 = L

    xw = x - L2 * np.cos(theta)
    yw = y - L2 * np.sin(theta)

    r2 = xw**2 + yw**2

    cosbeta = (r2 - L0**2 - L1**2)/(2*L0*L1)
    beta = np.arctan2(np.sqrt(1-cosbeta**2),cosbeta)

    alpha = np.arctan2(yw,xw) - arctan2(L1 * np.sin(beta), L0 + L1 * cosbeta)

    gamma = theta - alpha - beta

    return [float(alpha), float(beta), float(gamma)]






def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('env', help='input YAML file with environment')
    parser.add_argument('pdf', help='output pdf file')
    parser.add_argument('--export-car', type=str, default=None,
                        help='File to export planner data')
    parser.add_argument('--export-arm', type=str, default=None,
                        help='File to export planner data')
    args = parser.parse_args()

    with open(args.env, "r") as stream:
        env = yaml.safe_load(stream)

    env_min = env["environment"]["min"]
    env_max = env["environment"]["max"]
    corners = []
    for k, o in enumerate(env["environment"]["obstacles"]):
        if o["type"] == "box":
            p = o["pos"]
            s = o["size"]

            px = p[0]
            py = p[1]
            sx = s[0]/2
            sy = s[1]/2

            corners.append([px - sx, py - sy])
            corners.append([px+sx, py-sy])
            corners.append([px + sx, py + sy])
            corners.append([px - sx, py + sy])


        else:
            raise RuntimeError("Unknown obstacle type " + o["type"])
    corners = np.array(corners)
    start = env["motionplanning"]["start"]
    goal = env["motionplanning"]["goal"]
    path = np.array(env["motionplanning"]["solutionpath"])[:,:2]

    W = []
    B = []
    for i in range(len(path)-1):
        segW= []
        segB = []
        for j in range(0,len(corners),4):
            w, b = compute_hyperplane(corners[j:4+j], [path[i], path[i+1]])
            segW.append(w)
            segB.append(b)
        W.append(segW)
        B.append(segB)

    P = compute_control_points(path,W,B,env_min,env_max)

    plt.plot(P[:, 0], P[:, 1], 'x', label="Control Points")
    plt.plot(path[:, 0], path[:, 1], '--o', label="Path")

    t_vals = np.linspace(0, 1, 100)

    for i in range(len(path)-1):
        seg = np.array([bezier(P[3*i:3*i +4], t) for t in t_vals])
        plt.plot(seg[:, 0], seg[:, 1], label=f"Curve {i}")

    for i in range(0,len(corners),4):
        #print(i)
        plt.fill(corners[i:i+4, 0], corners[i:i+4, 1], alpha=0.2, label=f'O{int(i/4 +1)}')

    for segW,segB in zip(W,B):
        for w,b in zip(segW,segB):
            x,y = hyperplane(w,b)
            plt.plot(x,y,'--')

    plt.xlim(env_min[0], env_max[0])
    plt.ylim(env_min[1], env_max[1])
    plt.title("Bézier Path")
    plt.legend(prop = {'size' : 8}, loc = 'upper right')
    plt.grid(True)
    plt.savefig(args.pdf)
    #plt.show()

    #export to car
    if args.export_car:
        L = 3
        H = 1
        W = 1.5
        dt = 0.1
        states = []
        actions = []
        for i in range(len(path)-1):
            ctrP = P[3*i:3*i+4]
            for t in np.arange(0, 1, dt):
                x, y = bezier(ctrP, t)

                dx, dy = dBezier(ctrP, t)
                ddx, ddy = ddBezier(ctrP, t)
                theta = np.arctan2(dy, dx)
                v = np.sqrt(dx ** 2 + dy ** 2)
                v = np.clip(v, -0.5, 2)
                delta = np.arctan2(L * (ddy * dx - ddx * dy), max(v ** 3, 1e-5))
                states.append([float(x),float(y),float(theta)])
                actions.append([float(v),float(delta)])
        #last point at t=1
        x, y = bezier(ctrP, 1)
        dx, dy = dBezier(ctrP, 1)
        ddx, ddy = ddBezier(ctrP, 1)
        theta = np.arctan2(dy, dx)
        v = np.sqrt(dx ** 2 + dy ** 2)
        v = np.clip(v, -0.5, 2)
        delta = np.arctan2(L * (ddy * dx - ddx * dy), max(v ** 3, 1e-5))
        states.append([float(x), float(y), float(theta)])
        actions.append([float(v), float(delta)])

        plan = {
            'plan': {
                'type': 'car',
                'dt': dt,
                'L': L,
                'W': W,
                'H': H,
                'states': states,
                'actions': actions
            }
        }
        # Write to a YAML file
        with open(args.export_car, "w") as plan_file:
            yaml.safe_dump(plan, plan_file)

    #export to arm:
    if args.export_arm:
        L = [4,4,4]
        dt = 0.1
        states = []
        for i in range(len(path) - 1):
            ctrP = P[3 * i:3 * i + 4]
            for t in np.arange(0, 1, dt):
                x, y = bezier(ctrP, t)
                states.append(inverseKinematics([x,y,0],L))
        # last point at t=1
        x, y = bezier(ctrP, 1)
        states.append(inverseKinematics([x,y,0],L))
        plan = {
            'plan': {
                'type': 'arm',
                'L': L,
                'states': states,
            }
        }
        # Write to a YAML file
        with open(args.export_arm, "w") as plan_file:
            yaml.safe_dump(plan, plan_file)

if __name__ == "__main__":
    main()