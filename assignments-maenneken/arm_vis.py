import argparse
import yaml
import time
import numpy as np

# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('env', help='input YAML file with environment')
    parser.add_argument('plan', help='input YAML file with plan')
    parser.add_argument('--dt', type=float, default=0.1, help='sleeping time between frames')
    parser.add_argument('--output', default=None, help='output file with animation')
    args = parser.parse_args()

    with open(args.env, "r") as stream:
        env = yaml.safe_load(stream)

    with open(args.plan, "r") as stream:
        plan = yaml.safe_load(stream)

    vis = meshcat.Visualizer()

    for k, o in enumerate(env["environment"]["obstacles"]):
        if o["type"] == "box":
            p = o["pos"]
            s = o["size"]
            vis["obstacles"][str(k)].set_object(g.Box(s))
            vis["obstacles"][str(k)].set_transform(tf.translation_matrix(p))
        elif o["type"] == "cylinder":
            p = o["pos"]
            q = o["q"]
            r = o["r"]
            lz = o["lz"]
            vis["obstacles"][str(k)].set_object(g.Cylinder(lz, r))
            # NOTE: Additional transformation to match fcl
            vis["obstacles"][str(k)].set_transform(
                tf.translation_matrix(p).dot(
                    tf.quaternion_matrix(q)).dot(
                    tf.euler_matrix(np.pi / 2, 0, 0)))
        else:
            raise RuntimeError("Unknown obstacle type " + o["type"])

    L0, L1, L2 = plan["plan"]["L"]
    r = 0.04

    vis["arm"]["link0"].set_object(g.Cylinder(L0, r), g.MeshLambertMaterial(
        opacity=0.5))
    vis["arm"]["link1"].set_object(g.Cylinder(L1, r), g.MeshLambertMaterial(
        opacity=0.5))
    vis["arm"]["link2"].set_object(g.Cylinder(L2, r), g.MeshLambertMaterial(
        opacity=0.5))
    vis["arm"]["link0"].set_transform(tf.translation_matrix([L0/2, 0, 0]).dot(
                    tf.euler_matrix(0, 0, np.pi / 2)))
    vis["arm"]["link1"].set_transform(tf.translation_matrix([L0 + L1 / 2,0,0]).dot(
                    tf.euler_matrix(0, 0, np.pi / 2)))
    vis["arm"]["link2"].set_transform(tf.translation_matrix([L0+L1+L2/2, 0, 0]).dot(
                    tf.euler_matrix(0, 0, np.pi / 2)))

    anim = Animation()

    for k, (alpha,beta,gamma) in enumerate(plan["plan"]["states"]):
        with anim.at_frame(vis, k / args.dt) as frame:


            T0 = tf.euler_matrix(0,0, alpha)
            T1 = T0.dot(tf.translation_matrix([L0, 0, 0])).dot(tf.euler_matrix(0,0, beta))
            T2 = T1.dot(tf.translation_matrix([L1, 0, 0])).dot(tf.euler_matrix(0,0, gamma))

            #Note: we have to offset, because the cylinder is roted around its center, and we have to add 90deg because the object is roted the wrong way
            frame["arm"]["link0"].set_transform(T0.dot(tf.translation_matrix([L0 / 2, 0, 0])).dot(tf.euler_matrix(0, 0, np.pi / 2)))
            frame["arm"]["link1"].set_transform(T1.dot(tf.translation_matrix([L1/2, 0, 0])).dot(tf.euler_matrix(0, 0, np.pi / 2)))
            frame["arm"]["link2"].set_transform(T2.dot(tf.translation_matrix([L2/2, 0, 0])).dot(tf.euler_matrix(0, 0, np.pi / 2)))

    vis.set_animation(anim)

    if args.output is None:
        vis.open()
        time.sleep(1e9)
    else:
        res = vis.static_html()
        with open(args.output, "w") as f:
            f.write(res)


if __name__ == "__main__":
    main()
