import argparse
import yaml
import time
import numpy as np
import fcl
import ompl.base as ob
import ompl.geometric as og
import ompl.control as oc
import ompl.tools as tools
#import matplotlib.pyplot as plt
from numpy.random import uniform

from  nearest_neighbor import NearestNeighbor
from collisions import (
    collision_arm,
    setup_arm,
    setup_car,
    collision_car
)
from ompl_planner import (
    isStateValidCar,
    propagator
)
def make_sampler_allocator(sampler_type, si):
    if sampler_type == "uniform":
        return ob.ValidStateSamplerAllocator(lambda si_: ob.UniformValidStateSampler(si_))
    elif sampler_type == "bridge":
        return ob.ValidStateSamplerAllocator(lambda si_: ob.BridgeTestValidStateSampler(si_))
    elif sampler_type == "obstacle":
        return ob.ValidStateSamplerAllocator(lambda si_: ob.ObstacleBasedValidStateSampler(si_))
    elif sampler_type == "gaussian":
        return ob.ValidStateSamplerAllocator(lambda si_: ob.GaussianValidStateSampler(si_))
    else:
        raise ValueError(f"Unknown sampler type: {sampler_type}")
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('env', help='input YAML file with environment')
    parser.add_argument('benchmark', help='output log file of benchmark')
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

    if type != "car":
        raise RuntimeError("Only car supported " + type)

    dt = env["motionplanning"]["dt"]
    W = env["motionplanning"]["W"]
    H = env["motionplanning"]["H"]
    dist_type = 'se2'
    car = setup_car(L,W,H)
    goal_bias = env["hyperparameters"]["goal_bias"]
    goal_eps = env["hyperparameters"]["goal_eps"]
    timelimit = env["hyperparameters"]["timelimit"]

    #ompl Starts here

    space = ob.SE2StateSpace()  # (x,y,theta)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, min[0])
    bounds.setHigh(0, max[0])
    bounds.setLow(1, min[1])
    bounds.setHigh(1, max[1])
    space.setBounds(bounds)

    control_space = oc.RealVectorControlSpace(space, 2)
    control_bounds = ob.RealVectorBounds(2)
    control_bounds.setLow(0, -0.5)
    control_bounds.setHigh(0, +2.0)
    control_bounds.setLow(1, -np.pi / 6)
    control_bounds.setHigh(1, +np.pi / 6)
    control_space.setBounds(control_bounds)

    ss = oc.SimpleSetup(control_space)
    ss.setStateValidityChecker(
        ob.StateValidityCheckerFn(lambda s: isStateValidCar(car, manager, s))
    )
    ss.setStatePropagator(
        oc.StatePropagatorFn(lambda state, control, duration, target:
                             propagator(state, control, dt, target, L))
    )

    si = ss.getSpaceInformation()

    start_state = ob.State(space)
    start_state[0] = start[0]
    start_state[1] = start[1]
    start_state[2] = start[2]
    goal_state = ob.State(space)
    goal_state[0] = goal[0]
    goal_state[1] = goal[1]
    goal_state[2] = goal[2]

    ss.setStartAndGoalStates(start_state, goal_state, goal_eps)

    benchmark = tools.Benchmark(ss)

    query = benchmark.Request()
    query.runCount = 5
    query.maxTime = 1

    samplers = ["uniform", "bridge", "obstacle", "gaussian"]

    times = range(1, 11)
    mean_cost = []
    withoutBenchmark = False
    if withoutBenchmark:
        for t in times:
            for sampler in samplers:
                cost = []
                for i in range(5):
                    si = ss.getSpaceInformation()
                    si.setValidStateSamplerAllocator(make_sampler_allocator(sampler, si))
                    planner = oc.SST(si)
                    planner.setName(f"SST_{sampler}")
                    planner.setGoalBias(goal_bias)
                    ss.setPlanner(planner)
                    ss.setup()
                    ss.solve(t)
                    cost.append(ss.getSolutionPath().length())

                mean_cost.append(np.mean(cost))

        uniform_cost = mean_cost[0::4]
        bridge_cost = mean_cost[1::4]
        obstacle_cost = mean_cost[2::4]
        gaussian_cost = mean_cost[3::4]
        print(mean_cost)
        '''
        plt.plot(times, uniform_cost, label="Uniform")
        plt.plot(times, bridge_cost, label="Bridge")
        plt.plot(times, obstacle_cost, label="Obstacle")
        plt.plot(times, gaussian_cost, label="Gaussian")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("Cost")
        plt.show()
        '''

    else:
        for sampler in samplers:
            si = ss.getSpaceInformation()
            si.setValidStateSamplerAllocator(make_sampler_allocator(sampler, si))
            planner = oc.SST(si)
            planner.setName(f"SST_{sampler}")
            planner.setGoalBias(goal_bias)
            benchmark.addPlanner(planner)

        benchmark.benchmark(query)
        result = benchmark.getRecordedExperimentData()
        benchmark.saveResultsToFile(args.benchmark)





if __name__ == "__main__":
    main()
