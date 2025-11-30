import argparse
import yaml
import time
import numpy as np
import fcl

# visualization related
import meshcat.transformations as tf


def collision_arm(q, manager, arm, L, links):
    alpha, beta, gamma = q
    L0, L1, L2 = L
    link0, link1, link2 = links
    r = 0.04
    T0 = tf.euler_matrix(0, 0, alpha)
    T1 = T0.dot(tf.translation_matrix([L0, 0, 0])).dot(tf.euler_matrix(0, 0, beta))
    T2 = T1.dot(tf.translation_matrix([L1, 0, 0])).dot(tf.euler_matrix(0, 0, gamma))

    link0.setTransform(
        matrix_to_transform(T0.dot(tf.translation_matrix([L0 / 2, 0, 0])).dot(tf.euler_matrix(0, np.pi / 2, 0))))
    link1.setTransform(
        matrix_to_transform(T1.dot(tf.translation_matrix([L1 / 2, 0, 0])).dot(tf.euler_matrix(0, np.pi / 2, 0))))
    link2.setTransform(
        matrix_to_transform(T2.dot(tf.translation_matrix([L2 / 2, 0, 0])).dot(tf.euler_matrix(0, np.pi / 2, 0))))

    arm.update()

    req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
    rdata = fcl.CollisionData(request=req)

    manager.collide(arm, rdata, fcl.defaultCollisionCallback)
    return rdata.result.is_collision

def setup_arm(L):
    L0, L1, L2 = L
    r = 0.05
    R = tf.euler_matrix(0, np.pi / 2, 0)[:3][:3]
    cyl0 = fcl.Cylinder(r, L0)
    cyl1 = fcl.Cylinder(r, L1)
    cyl2 = fcl.Cylinder(r, L2)
    link0 = fcl.CollisionObject(cyl0, fcl.Transform(np.array([L0 / 2, 0, 0])))
    link1 = fcl.CollisionObject(cyl1, fcl.Transform(np.array([L0 + L1 / 2, 0, 0])))
    link2 = fcl.CollisionObject(cyl2, fcl.Transform(np.array([L0 + L1 + L2 / 2, 0, 0])))
    link0.setRotation(R)
    link1.setRotation(R)
    link2.setRotation(R)

    arm = fcl.DynamicAABBTreeCollisionManager()
    arm.registerObjects([link0, link1, link2])
    arm.setup()
    return arm, [link0, link1, link2]

def collision_car(car, manager, q):
    x,y, theta = q
    car.setTransform(fcl.Transform([x, y, 0.5]))
    R = tf.euler_matrix(0, 0, theta)[:3][:3]  # is this right????
    car.setRotation(R)
    req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
    rdata = fcl.CollisionData(request=req)

    manager.collide(car, rdata, fcl.defaultCollisionCallback)
    return rdata.result.is_collision

def setup_car(L,W,H):
    box = fcl.Box(L, W, H)
    car = fcl.CollisionObject(box)
    return car

def wrap_angle_diff(a, b):
    return np.arctan2(np.sin(b - a), np.cos(b - a))

def angle_interp(q1, q2, alpha):
    return (q1 + alpha * wrap_angle_diff(q1, q2)) % (2 * np.pi)

def edge_in_collision(x1, x2, manager, arm, L, links, step_size=0.05):
    x1 = np.array(x1)
    x2 = np.array(x2)
    distance = np.linalg.norm(wrap_angle_diff(x1, x2))
    steps = max(int(distance / step_size), 1)

    for i in range(1, steps):
        alpha = i / steps
        q = angle_interp(x1, x2, alpha)
        if collision_arm(q, manager, arm, L, links):
            return True
    return False

def matrix_to_transform(M):
    """Convert a 4x4 matrix to fcl.Transform."""
    rot = M[:3, :3]
    trans = M[:3, 3]
    return fcl.Transform(rot, trans)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('env', help='input YAML file with environment')
    parser.add_argument('plan', help='input YAML file with plan')
    parser.add_argument('collisions', help='output YAML file with collisions')
    parser.add_argument('--dt', type=float, default=0.1, help='sleeping time between frames')
    args = parser.parse_args()

    with open(args.env, "r") as stream:
        env = yaml.safe_load(stream)

    with open(args.plan, "r") as stream:
        plan = yaml.safe_load(stream)

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
    collisions = []
    #We have a car
    if(plan["plan"]["type"] == "car"):
        L = plan["plan"]["L"]
        W = plan["plan"]["W"]
        H = plan["plan"]["H"]
        box = fcl.Box(L,W,H)
        car = fcl.CollisionObject(box)
        car = setup_car(L, W, H)
        for k, (x, y, theta) in enumerate(plan["plan"]["states"]):

            # Set the car's transform
            car.setTransform(fcl.Transform([x, y, 0.5]))
            R = tf.euler_matrix(0, 0, theta)[:3][:3] #is this right????
            car.setRotation(R)
            req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
            rdata = fcl.CollisionData(request=req)

            manager.collide(car, rdata, fcl.defaultCollisionCallback)
            collisions.append(rdata.result.is_collision)


    elif(plan["plan"]["type"] == "arm"):
        L0, L1, L2 = plan["plan"]["L"]
        r = 0.04
        R = tf.euler_matrix(0,np.pi / 2, 0)[:3][:3]
        cyl0 = fcl.Cylinder(r,L0)
        cyl1 = fcl.Cylinder(r,L1)
        cyl2 = fcl.Cylinder(r,L2)
        link0 = fcl.CollisionObject(cyl0,fcl.Transform(np.array([L0/2, 0, 0])))
        link1 = fcl.CollisionObject(cyl1,fcl.Transform(np.array([L0 + L1 / 2,0,0])))
        link2 = fcl.CollisionObject(cyl2,fcl.Transform(np.array([L0 + L1 + L2/ 2,0,0])))
        link0.setRotation(R)
        link1.setRotation(R)
        link2.setRotation(R)

        arm = fcl.DynamicAABBTreeCollisionManager()
        arm.registerObjects([link0,link1,link2])
        arm.setup()
        for k, (alpha,beta,gamma) in enumerate(plan["plan"]["states"]):

            T0 = tf.euler_matrix(0, 0, alpha)
            T1 = T0.dot(tf.translation_matrix([L0, 0, 0])).dot(tf.euler_matrix(0, 0, beta))
            T2 = T1.dot(tf.translation_matrix([L1, 0, 0])).dot(tf.euler_matrix(0, 0, gamma))

            link0.setTransform(matrix_to_transform(T0.dot(tf.translation_matrix([L0 / 2, 0, 0])).dot(tf.euler_matrix(0,np.pi / 2, 0))))
            link1.setTransform(matrix_to_transform(T1.dot(tf.translation_matrix([L1/2, 0, 0])).dot(tf.euler_matrix(0,np.pi / 2, 0))))
            link2.setTransform(matrix_to_transform(T2.dot(tf.translation_matrix([L2/2, 0, 0])).dot(tf.euler_matrix(0,np.pi / 2, 0))))

            arm.update()

            req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
            rdata = fcl.CollisionData(request=req)

            manager.collide(arm, rdata, fcl.defaultCollisionCallback)
            collisions.append(rdata.result.is_collision)

    # Write to a YAML file
    with open(args.collisions, "w") as collisions_file:
        yaml.safe_dump({"collisions": collisions}, collisions_file)


if __name__ == "__main__":
    main()
