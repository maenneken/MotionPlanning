"""
e have u = (s, ϕ) ∈ U and q = (x, y , θ) ∈ Q,
where s is the speed, ϕ the steering wheel angle,
x, y is the position, and θ is the orientation. The
dynamics ˙q = f(q, u) are:
Dx = s cos θ
Dy = s sin θ ˙
Dθ = s/L tan ϕ
"""
import argparse
import yaml
import time
import numpy as np
from tornado.autoreload import start



def car_dyn(q, action, dt, L):
    x, y, theta = q
    s, phi = action

    dx = s * np.cos(theta)
    dy = s * np.sin(theta)
    dtheta = s / L *np.tan(phi)

    #Euler Integration
    x = x + dx *dt
    y = y + dy *dt
    theta = theta + dtheta * dt

    return [float(x),float(y),float(theta)]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('actions', help='input YAML file with actions')
    parser.add_argument('plan', help='input YAML file for the plan')
    parser.add_argument('--dt', type=float, default=0.1, help='sleeping time between frames')
    parser.add_argument('--output', default=None, help='output file with calculated plan')
    args = parser.parse_args()

    with open(args.actions, "r") as stream:
        actions = yaml.safe_load(stream)


    dt = actions["dt"]
    L = actions["L"]
    W = actions["W"]
    H = actions["H"]
    start_state = actions["start"]
    actions = actions["actions"]
    states = [start_state]
    #calk plan
    for action in actions:
        states.append(car_dyn(states[-1], action, dt, L))


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
    with open(args.plan, "w") as plan_file:
        yaml.safe_dump(plan, plan_file)

if __name__ == "__main__":
    main()