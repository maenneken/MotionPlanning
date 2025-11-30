import argparse
import yaml
import time
import numpy as np
import fcl
from numpy.random import uniform

from  nearest_neighbor import NearestNeighbor
from collisions import (
    collision_arm,
    setup_arm,
    setup_car,
    collision_car
)

class RRT(NearestNeighbor):
    neighbors = {}
    def __init__(self, goal, start, points, distType):
        super().__init__(points, distType)
        self.goal = goal
        self.start = start
        self.addConfiguration(self.start,[])

    def addConfiguration(self, q, neighbors):
        #the parent of q should be neighbors[q][0]
        super().addConfiguration(q)
        self.neighbors[tuple(q)] = neighbors

    def getPath(self,q):
        current = np.array(q).tolist()
        path = [current]
        while not np.array_equal(current, self.start):
            current = self.neighbors[tuple(current)][0]  # parent is always first
            path.append(np.array(current).tolist())

        path.reverse()
        return path

def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

def steer_arm(xnear, xrand, mu, manager, arm, L, links, step_size=0.01):
    xnear = np.array(xnear)
    xrand = np.array(xrand)

    direction = np.arctan2(np.sin(xrand - xnear), np.cos(xrand - xnear))
    direction = direction / np.linalg.norm(direction)  # normalize direction vector

    distance = 0.0
    q_prev = np.array(xnear)

    while distance < mu:
        q_next = (q_prev + step_size * direction) % (2 * np.pi)
        distance += step_size

        if collision_arm(q_next, manager, arm, L, links):
            return None if np.allclose(q_prev, xnear) else q_prev  # If first step collides, return None
        q_prev = q_next

    return q_prev

def steer_car(xnear, xrand, L, dt):
    s_min = -0.5
    s_max = 2
    phi_max = np.pi/6

    xnear = np.array(xnear)  # [x, y, theta]

    dx = xrand[0] - xnear[0]
    dy = xrand[1] - xnear[1]
    heading_to_target = np.arctan2(dy, dx)

    dtheta = wrap_angle(heading_to_target - xnear[2])
    distance = np.cos(dtheta) * np.linalg.norm([dx, dy])
    s = np.clip(distance, s_min, s_max)
    phi = np.clip(dtheta, -phi_max, phi_max)


    # avoid zero velocity (robot doesn't move)
    if abs(s) < 1e-2:
        s = 1e-2 * np.sign(s) if s != 0 else 1e-2
    x = np.array(xnear, dtype=float)
    theta = x[2]
    x[0] += s * np.cos(theta) * dt
    x[1] += s * np.sin(theta) * dt
    x[2] += (s / L) * np.tan(phi) * dt
    x[2] = wrap_angle(x[2])

    controls = (s, phi)

    return x, controls

def get_actios_from_path(actionDict, path):
    actions = []
    for p in path[1:]:
        actions.append(np.array(actionDict[tuple(p)]).tolist())
    return actions

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('env', help='input YAML file with environment')
    parser.add_argument('plan', help='output YAML file with plan')
    parser.add_argument('--dt', type=float, default=0.1, help='sleeping time between frames')
    args = parser.parse_args()

    with open(args.env, "r") as stream:
        env = yaml.safe_load(stream)

    manager = fcl.DynamicAABBTreeCollisionManager()
    objects = []
    for k, o in enumerate(env["environment"]["obstacles"]):
        if o["type"] == "box":
            p = o["pos"]
            s = o["size"]

            box = fcl.Box(*s)
            obj= fcl.CollisionObject(box,fcl.Transform(np.array(p)))
            objects.append(obj)

        elif o["type"] == "cylinder":
            p = o["pos"]
            q = o["q"]
            r = o["r"]
            lz = o["lz"]
            cyl = fcl.Cylinder(r,lz)
            obj = fcl.CollisionObject(cyl,fcl.Transform(np.array(q),np.array(p)))
            objects.append(obj)

        else:
            raise RuntimeError("Unknown obstacle type " + o["type"])

    manager.registerObjects(objects)
    manager.setup()
    min = env["environment"]["min"]
    max = env["environment"]["max"]

    goal = env["motionplanning"]["goal"]
    start = env["motionplanning"]["start"]
    L = env["motionplanning"]["L"]
    type = env["motionplanning"]["type"]

    if type == "arm":
        dist_type = 'angles'
        arm, links = setup_arm(L)
    elif type == "car":
        dt = env["motionplanning"]["dt"]
        W = env["motionplanning"]["W"]
        H = env["motionplanning"]["H"]
        dist_type = 'se2'
        car = setup_car(L,W,H)
    goal_bias = env["hyperparameters"]["goal_bias"]
    goal_eps = env["hyperparameters"]["goal_eps"]
    timelimit = env["hyperparameters"]["timelimit"]
    mu = 0.2  # max step size (1deg)


    rrt = RRT(goal,start,[], dist_type)
    found = False
    start_time = time.time()
    actionDict= {}

    while not found and (time.time() - start_time) < timelimit:

        if np.random.rand() < goal_bias:
            sample = goal
        elif type =="arm":
            sample = np.random.uniform(-np.pi, np.pi, size=3)
        elif type == "car":
            sample = np.array([
                np.random.uniform(min[0], max[0]),
                np.random.uniform(min[1], max[1]),
                np.random.uniform(-np.pi, np.pi)
            ])
        neighbor = rrt.nearestK(sample, 1)
        if not neighbor:
            continue
        xnear = neighbor[0]

        if type == "arm":
            xnew = steer_arm(xnear,sample, mu, manager, arm, L, links)
        elif type == "car":
            xnew, action = steer_car(xnear,sample,L, dt)
            actionDict[tuple(xnew)] = action
            if collision_car(car,manager,xnew):
                xnew = None
        if xnew is None:
            continue
        rrt.addConfiguration(xnew, [xnear])
        rrt.neighbors[tuple(xnear)].append(xnew)

        #we are at goal region
        if rrt.distance(xnew, rrt.goal) < goal_eps:
            print("found a path")
            path = rrt.getPath(xnew)
            found = True

    if (time.time() - start_time) > timelimit:
        print("nothing found time is up")
        path = rrt.getPath(rrt.nearestK(rrt.goal,1)[0])

    print(path)
    if(type=="arm"):
        plan = {
                'plan': {
                'type': 'arm',
                'L': L,
                'states': path
            }
        }

    elif(type == "car"):
        actions = get_actios_from_path(actionDict,path)
        plan = {
                'plan': {
                'type': 'car',
                'L': L,
                'W': W,
                'H': H,
                'dt': dt,
                'states': path,
                'actions': actions
            }
        }
    # Write to a YAML file
    with open(args.plan, "w") as plan_file:
        yaml.safe_dump(plan, plan_file)
if __name__ == "__main__":
    main()
