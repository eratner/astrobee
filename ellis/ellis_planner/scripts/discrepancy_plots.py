import argparse
import rosbag
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import math


def exp_likelihood(arg, thresh, param):
    return 1.0 / (1.0 + math.exp(arg/param - 5.0))

    # if arg >= thresh:
    #     return 1.0

    # return math.exp(-(arg - thresh)**2 / param**2)


def plot_likelihood_func():
    # mpl.rcParams['text.usetex'] = True
    # mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}']

    fig, ax = plt.subplots(figsize=(10, 5))

    dists = [0.001 * t for t in range(150)]
    likelihoods = [exp_likelihood(d, 0.1, 0.01) for d in dists]
    ax.plot(dists, likelihoods)
    ax.set_xlim([-0.15, 0.15])
    ax.set_ylim([-0.01, 1.01])
    # ax.set_xlabel("Error")
    # ax.set_ylabel("Discrepancy Likelihood")

    ax.set_xlabel("$\Vert x' - \hat{f}(x, u) \Vert$")
    ax.set_ylabel("$P(x, u, x' \mid$ discrepancy at $(x, u))$")

    plt.show()


def read_bagfile(bagfile):
    thresh = 0.050 # Discrepancy threshold

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
            print("x: {}, u: {}, e: {}".format(
                states[-1], controls[-1], np.linalg.norm(errors[-1])))


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

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_xlim([-0.05, 0.15])
        ax.set_ylim([-0.10, 0.10])
        ax.scatter(desired_xs, desired_ys, color='black')
        # ax.scatter(actual_xs, actual_ys, color='red')
        ax.scatter(actual_xs_ok, actual_ys_ok, color='green', marker='o')
        ax.scatter(actual_xs_not_ok, actual_ys_not_ok, color='red', marker='o')

        plt.show()

    bag.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    read_bagfile(args.bag)
