import argparse
import yaml
import time
import numpy as np
import fcl
import ompl.base as ob
import ompl.geometric as og
import ompl.control as oc

from numpy.random import uniform

from  nearest_neighbor import NearestNeighbor
from collisions import (
    collision_arm,
    setup_arm,
    setup_car,
    collision_car
)
def pathToList(path):
    list =[]
    for s in path:
        list.append([s[0].value,s[1].value,s[2].value])
    return list

def pathToListSE2(path):
    list =[]
    for s in path:
        list.append([s.getX(),s.getY(),s.getYaw()])
    return list

def controlsToList(controls):
    list = []
    for c in controls:
        list.append([c[0],c[1]])
    return list

def isStateValidArm(q, manager, arm, L, links):
    return not collision_arm(q, manager, arm, L, links)

def isStateValidCar(car, manager, q):
    x = q.getX()
    y = q.getY()
    theta = q.getYaw()
    return not collision_car(car, manager,[x,y,theta])

def propagator(state, control, dt, target,L):
  x = state.getX()
  y = state.getY()
  theta = state.getYaw()

  s = control[0]
  phi = control[1]

  dx = s * np.cos(theta)
  dy = s * np.sin(theta)
  dtheta = s / L * np.tan(phi)

  # Euler Integration
  x = x + dx * dt
  y = y + dy * dt
  theta = theta + dtheta * dt

  target.setX(x)
  target.setY(y)
  target.setYaw(theta)

  return target

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('env', help='input YAML file with environment')
    parser.add_argument('plan', help='output YAML file with plan')
    parser.add_argument('--export-planner-data', type=str, default=None,
                        help='File to export planner data (tree structure)')
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


    if "planner" in env["hyperparameters"]:
        planner_type = env["hyperparameters"]["planner"]
    else:
        planner_type = "rrt"

    #ompl Starts here
    ndof = 3

    if(type == "arm"):
        space = ob.CompoundStateSpace()
        space.addSubspace(ob.SO2StateSpace(), 1.0) #joint1
        space.addSubspace(ob.SO2StateSpace(), 1.0) #joint2
        space.addSubspace(ob.SO2StateSpace(), 1.0) #joint3

        ss = og.SimpleSetup(space)
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(
            lambda s: isStateValidArm([s[i].value  for i in range(3)], manager, arm, L, links)))

        if planner_type == "rrt":
            planner = og.RRT(ss.getSpaceInformation())
            planner.setGoalBias(goal_bias)
        elif planner_type == "rrt*":
            planner = og.RRTstar(ss.getSpaceInformation())
            planner.setGoalBias(goal_bias)
        elif planner_type == "rrt-connect":
            planner = og.RRTConnect(ss.getSpaceInformation())
        else:
            raise ValueError(f"Unsupported planner: {planner_type}")

        ss.setPlanner(planner)
        start_state = ob.State(space)
        start_state[0] = start[0]
        start_state[1] = start[1]
        start_state[2] = start[2]
        goal_state = ob.State(space)
        goal_state[0] = goal[0]
        goal_state[1] = goal[1]
        goal_state[2] = goal[2]

        ss.setStartAndGoalStates(start_state, goal_state, goal_eps)
        ss.solve(timelimit)
        path = ss.getSolutionPath()
        print(path)

        plan = {
                'plan': {
                'type': 'arm',
                'L': L,
                'states': pathToList(path.getStates())
            }
        }

    elif(type == "car"):

        space = ob.SE2StateSpace() #(x,y,theta)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, min[0])
        bounds.setHigh(0,max[0])
        bounds.setLow(1, min[1])
        bounds.setHigh(1, max[1])
        space.setBounds(bounds)

        control_space = oc.RealVectorControlSpace(space, 2)
        control_bounds = ob.RealVectorBounds(2)
        control_bounds.setLow(0, -0.5)
        control_bounds.setHigh(0, +2.0)
        control_bounds.setLow(1, -np.pi/6)
        control_bounds.setHigh(1, +np.pi/6)
        control_space.setBounds(control_bounds)

        si = oc.SpaceInformation(space, control_space)
        si.setStatePropagator(oc.StatePropagatorFn(lambda state, control, duration, target: propagator(state, control, dt, target, L)))
        si.setStateValidityChecker(ob.StateValidityCheckerFn(
            lambda s: isStateValidCar(car,manager,s)))

        start_state = ob.State(space)
        start_state[0] = start[0]
        start_state[1] = start[1]
        start_state[2] = start[2]
        goal_state = ob.State(space)
        goal_state[0] = goal[0]
        goal_state[1] = goal[1]
        goal_state[2] = goal[2]

        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start_state, goal_state, goal_eps)


        if planner_type == "rrt":
            planner = oc.RRT(si)
            planner.setGoalBias(goal_bias)
        elif planner_type == "sst":
            planner = oc.SST(si)
            planner.setGoalBias(goal_bias)
        else:
            raise ValueError(f"Unsupported planner: {planner_type}")

        planner.setProblemDefinition(pdef)
        planner.setup()
        planner.solve(timelimit)

        path = pdef.getSolutionPath()
        path.interpolate()
        print(path)
        plan = {
                'plan': {
                'type': 'car',
                'L': L,
                'W': W,
                'H': H,
                'dt': dt,
                'states': pathToListSE2(path.getStates()),
                'actions': controlsToList(path.getControls()),
            }
        }
    # Write to a YAML file
    with open(args.plan, "w") as plan_file:
        yaml.safe_dump(plan, plan_file)

    #Export planner tree
    if args.export_planner_data is not None and type == "car":
        pdata = ob.PlannerData(si)
        planner.getPlannerData(pdata)
        print(f"Tree with {pdata.numVertices()} Vertices and {pdata.numEdges()} Edges")
        with open(args.export_planner_data, "w") as f:
            f.write(f"{pdata.numVertices()}\n")
            for i in range(pdata.numVertices()):
                v = pdata.getVertex(i)
                s = v.getState()
                x, y, theta = s.getX(), s.getY(), s.getYaw()
                f.write(f"{x} {y} {theta}\n")

            for i in range(pdata.numVertices()):
                for j in range(pdata.numVertices()):
                    if pdata.edgeExists(i, j):
                        f.write(f"{i} {j}\n")

if __name__ == "__main__":
    main()
