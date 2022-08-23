import argparse
import rosbag
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import math


class DiscrepancyClassifier:
    NOT_DISCREPANCY = -1
    DISCREPANCY = 1

    def __init__(self, k=5):
        self._k = k
        self._observations = []

    def add_observation(self, x, u, is_discrepancy=True):
        self._observations.append(
            (x, u, self.DISCREPANCY if is_discrepancy else self.NOT_DISCREPANCY))

    def dist(self, xa, ua, xb, ub):
        state_dist = np.linalg.norm(xa - xb)
        control_angle = np.arccos(np.dot(ua, ub) /
                                  (np.linalg.norm(ua) * np.linalg.norm(ub)))
        # TODO(eratner) Determine the relative weightings
        # return (1.0 * state_dist + 0.5 * control_angle)
        return (1.0 * state_dist + 0.25 * control_angle)

    def prob_func_v1(self, arg, param=1.0):
        return np.exp(-arg**2 / param**2)

    def get_discrepancy_prob_v1(self, x, u):
        if len(self._observations) == 0:
            return 0.0

        min_dist = 1e9
        for o in self._observations:
            dist = self.dist(o[0], o[1], x, u)
            min_dist = min(min_dist, dist)

        return self.prob_func_v1(min_dist, 0.1)


def read_bagfile(bagfile):
    # thresh = 0.050 # Discrepancy threshold
    thresh = 0.025

    print("Reading from {}...".format(bagfile))
    bag = rosbag.Bag(bagfile)
    for topic, msg, t in bag.read_messages(topics=['/exp/control_history']):
        # Construct the data set.
        states = []
        controls = []
        errors = []

        for i in range(len(msg.desired_pose) - 1):
            actual_pose = msg.actual_pose[i]
            desired_pose = msg.desired_pose[i + 1]
            next_actual_pose = msg.actual_pose[i + 1]

            # Compute the control (direction).
            u = np.array([desired_pose.position.x - actual_pose.position.x,
                          desired_pose.position.y - actual_pose.position.y])
            u_norm = np.linalg.norm(u)
            if u_norm < 1e-4:
                # Skip if the control is zero.
                continue

            states.append(np.array([actual_pose.position.x,
                                    actual_pose.position.y]))
            controls.append(u / u_norm)
            errors.append(np.array([
                desired_pose.position.x - next_actual_pose.position.x,
                desired_pose.position.y - next_actual_pose.position.y]))
            print("x: {}, u: {}, e: {}, u (unnorm): {}".format(
                states[-1], controls[-1], np.linalg.norm(errors[-1]), u))


        disc_classifier = DiscrepancyClassifier()
        for x, u, e in zip(states, controls, errors):
            if np.linalg.norm(e) > thresh:
                disc_classifier.add_observation(x, u)

        ########################################################################
        desired_xs = [p.position.x for p in msg.desired_pose]
        desired_ys = [p.position.y for p in msg.desired_pose]

        actual_xs = [p.position.x for p in msg.actual_pose]
        actual_ys = [p.position.y for p in msg.actual_pose]

        actual_xs_ok = []
        actual_ys_ok = []
        actual_xs_not_ok = []
        actual_ys_not_ok = []

        errors = [np.array([pd.position.x - pa.position.x,
                            pd.position.y - pa.position.y])
                  for pd, pa in zip(msg.desired_pose, msg.actual_pose)]
        for i in range(len(errors)):
            if np.linalg.norm(errors[i]) <= thresh:
                # No discrepancy.
                actual_xs_ok.append(msg.actual_pose[i].position.x)
                actual_ys_ok.append(msg.actual_pose[i].position.y)
            else:
                # Discrepancy.
                actual_xs_not_ok.append(msg.actual_pose[i].position.x)
                actual_ys_not_ok.append(msg.actual_pose[i].position.y)
        ########################################################################

        U = [np.array([-1.0, 0.0]), np.array([1.0, 0.0]),
             np.array([0.0, -1.0]), np.array([0.0, 1.0])]
        for u in U:
            probs = np.zeros((40, 40))

            min_x = -0.05
            step_x = 0.01
            min_y = -0.20
            step_y = 0.01

            fig, ax = plt.subplots(figsize=(10, 10))
            ax.set_title("$u = ({}, {})$".format(u[0], u[1]))
            ax.set_xlim([min_x, min_x + 40 * step_x])
            ax.set_ylim([min_y, min_y + 40 * step_y])
            ax.set_xlabel("$x$ ($m$)")
            ax.set_ylabel("$y$ ($m$)")

            px = []
            py = []
            pz = []

            for i in range(40):
                for j in range(40):
                    x = np.array([min_x + i * step_x,
                                  min_y + j * step_y])
                    prob = disc_classifier.get_discrepancy_prob_v1(x, u)

                    px.append(x[0])
                    py.append(x[1])
                    pz.append(prob)

                    # b = max(0.0, 1.0 - prob)
                    # r = max(0.0, prob - 1.0)
                    # g = 1.0 - b - r
                    # ax.scatter([x[0]], [x[1]], color=(r, g, b))

            ax.scatter(px, py, s=120, marker='s', edgecolors='none', alpha=0.5, c=pz, cmap='jet', vmin=0.0, vmax=1.0)
            ax.scatter(desired_xs, desired_ys, color='black', label="Reference")
            ax.scatter(actual_xs_ok, actual_ys_ok, color='green', marker='o', label="Actual (Ok)")
            ax.scatter(actual_xs_not_ok, actual_ys_not_ok, color='red', marker='o', label="Actual (Not Ok)")

            # ax.imshow(probs, cmap='hot', interpolation='nearest', origin='lower')
            # hm = ax.pcolor(probs)
            # plt.colorbar(hm)
            plt.show()

        ########################################################################
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_title("Discrepancy threshold = {}".format(thresh))
        ax.set_xlim([-0.05, 0.35])
        ax.set_ylim([-0.20, 0.20])
        ax.set_xlabel("$x$ ($m$)")
        ax.set_ylabel("$y$ ($m$)")
        ax.scatter(desired_xs, desired_ys, color='black', label="Reference")
        # ax.scatter(actual_xs, actual_ys, color='red')
        ax.scatter(actual_xs_ok, actual_ys_ok, color='green', marker='o', label="Actual (Ok)")
        ax.scatter(actual_xs_not_ok, actual_ys_not_ok, color='red', marker='o', label="Actual (Not Ok)")
        ax.legend()

        plt.show()
        ########################################################################

    bag.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    read_bagfile(args.bag)
