import argparse
import numpy as np
import yaml


class NearestNeighbor:

    def __init__(self, points, distType):
        self.distType = distType
        self.points = points

    def distance(self,q,p):
        p = np.array(p)
        q = np.array(q)
        if self.distType == 'l2':
            return np.sqrt(np.sum((np.array(q) - np.array(p))**2))
        elif self.distType == 'angles':
            return np.linalg.norm(np.arctan2(np.sin(q-p), np.cos(q-p)))
        elif self.distType == 'se2':
            p1, a1 = q[:2], q[2]
            p2, a2 = p[:2], p[2]
            pos_dist = np.linalg.norm(p1 - p2)
            angle_dist = abs(np.arctan2(np.sin(a1-a2), np.cos(a1-a2)))
            return pos_dist + angle_dist
        else:
            raise ValueError(f"Unknown distance type: {self.distType}")
    def distances(self, q, points):
        dists = []
        for point in points:
            dists.append((self.distance(q,point),point))
        return dists

    def addConfiguration(self,q):
        self.points.append(q)

    def nearestK(self,q, k):
        dists = self.distances(q,self.points)
        dists.sort(key=lambda x: x[0])
        bestPoints = [point for _, point in dists[:k]]
        return bestPoints

    def nearestR(self,q, r):
        dists = self.distances(q, self.points)
        dists.sort(key=lambda x: x[0])
        bestPoints = [point for dist, point in dists if dist <= r]
        return bestPoints

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('queries', help='input YAML file for the nn queries')
    parser.add_argument('solution', help='output YAML file with solutions for queries')
    args = parser.parse_args()

    with open(args.queries, "r") as stream:
        queries = yaml.safe_load(stream)

    nn = NearestNeighbor([], queries["distance"])
    for config in queries["configurations"]:
        nn.addConfiguration(config)
    results = []

    for query in queries["queries"]:
        if query["type"]=="nearestK":
            results.append(nn.nearestK(query["q"],query["k"]))
        elif query["type"]=="nearestR":
            results.append(nn.nearestR(query["q"],query["r"]))

    with open(args.solution, "w") as solution_file:
        yaml.safe_dump({"results": results}, solution_file)

    with open(args.solution, "r") as solution_file:
        results = yaml.safe_load(solution_file)
    print(results)
if __name__ == "__main__":
    main()