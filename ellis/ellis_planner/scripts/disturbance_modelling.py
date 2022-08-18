import argparse
import rosbag
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.patches import Ellipse
from mpl_toolkits import mplot3d
import numpy as np
import scipy.linalg


class GPModel:
    def __init__(self):
        self._params = {
            'D': 1,     # Input dimension.
            'v0': 0.01, # Variance of Gaussian white noise added to model.
            'w': [1.0], # Hyperparameter of covariance function.
            'v1': 1.0,  # Hyperparameter of covariance function.
        }

    def cov_func(self, xp, xq):
        #print("xp = {}, xq = {}".format(xp, xq))
        return (self._params['v1'] * np.exp(
            -0.5 * sum([(xp[d] - xq[d])**2 / self._params['w'][d]**2
                        for d in range(self._params['D'])])))

    def train(self, inputs, targets):
        N = len(inputs)
        assert len(targets) == N, "targets and inputs should have same size!"

        # self._training_inputs = inputs
        self._training_inputs = [x.reshape((self._params['D'], 1))
                                 for x in inputs]
        self._y = np.array(targets).reshape((N, 1))

        self._K = np.zeros((N, N))
        for i in range(N):
            for j in range(N):
                self._K[i, j] = self.cov_func(inputs[i], inputs[j])

        self._K += (self._params['v0'] * np.eye(N))

    def mean_func(self, x):
        # TODO Do this in train()
        K_inv_y = scipy.linalg.solve(self._K, self._y, sym_pos=True)
        k = self.get_k(x)
        return k.T.dot(K_inv_y)

    def var_func(self, x):
        k = self.get_k(x)
        K_inv_k = scipy.linalg.solve(self._K, k, sym_pos=True)
        return (self.cov_func(x, x) - k.T.dot(K_inv_k))

    def get_k(self, x):
        N = len(self._training_inputs)
        k = np.zeros((N, 1))
        for i in range(N):
            #print("xi = {}, x = {}".format(self._training_inputs[i], x))
            k[i, 0] = self.cov_func(
                self._training_inputs[i], x)

        return k

    def get_first_deriv_mean_func(self, x):
        # TODO Do this in train()
        K_inv_y = scipy.linalg.solve(self._K, self._y, sym_pos=True)
        k = self.get_k(x)
        N = len(self._training_inputs)

        deriv = np.zeros((self._params['D'], 1))
        for d in range(self._params['D']):
            xd = np.array([xi[d]
                           for xi in self._training_inputs]).reshape((N, 1))
            deriv[d, 0] = np.dot(
                (self._params['w'][d] * np.multiply((xd - x[d]), k)).T , K_inv_y)

        return deriv

    def get_second_deriv_var_func(self, x):
        # TODO Avoid computing the inverse of K
        K_inv = np.linalg.inv(self._K)
        k = self.get_k(x)
        N = len(self._training_inputs)

        deriv = np.zeros((self._params['D'], self._params['D']))
        for d in range(self._params['D']):
            for e in range(self._params['D']):
                xd = np.array([xi[d]
                           for xi in self._training_inputs]).reshape((N, 1))
                xe = np.array([xi[e]
                           for xi in self._training_inputs]).reshape((N, 1))

                deriv[d, e] = -2.0 * self._params['w'][d] * self._params['w'][e] * (
                    np.multiply((xd - x[d]), k).T.dot(K_inv.dot(
                        np.multiply(xe - x[e], k))) +
                    np.multiply(xd - x[d], np.multiply(xe - x[e], k)).T.dot(
                        K_inv.dot(k)))
                if d == e:
                    deriv[d, e] += (
                        2.0 * self._params['w'][d] * k.T.dot(K_inv.dot(k)))

        return deriv


class LinearDynamics:
    def __init__(self, A, B, d):
        self._A = A
        self._B = B
        self._d = d

    def step(self, x, u, disturbance=False):
        x_next = self._A.dot(x) + self._B.dot(u)
        if disturbance:
            pass # TODO
        return x_next

    def predict(self, x_start, us):
        z_start = np.concatenate([x_start.flatten(), us[0].flatten()])
        D = len(z_start)
        D_state = len(x_start)

        z_pred_mean = [z_start.reshape((D, 1))]
        z_pred_cov = [np.zeros((D, D))]

        A = np.eye(D)
        A[:D_state, :D_state] = self._A
        A[:D_state, D_state:] = self._B

        #print("A = {}".format(A))

        for t in range(1, len(us) + 1):
            #print("***** t = {}".format(t))
            mean = z_pred_mean[-1]
            #print("  mean before = {}".format(mean))
            mean[D_state:, 0] = us[t - 1].flatten()
            cov = z_pred_cov[-1]
            #print("  mean = {}, cov = {}".format(mean, cov))

            # Predict the mean.
            d = np.zeros((D, 1))
            for i in range(len(self._d)):
                d[i, 0] = float(self._d[i].mean_func(mean))

            tmp = A.dot(mean)
            #print("mean = {}, A.dot(mean) = {}, d = {}".format(mean, tmp, d))
            next_mean = A.dot(mean) + d
            #print("next_mean = {}".format(next_mean))
            z_pred_mean.append(next_mean)

            # Predict the covariance.
            next_cov = np.zeros((D, D))
            if t == 1:
                for i in range(len(self._d)):
                    #print("mean = {}".format(mean))
                    next_cov[i, i] = float(self._d[i].var_func(mean))
            else:
                next_cov = A.dot(cov.dot(A.T)).reshape((D, D))

                vs = []
                for d in self._d:
                    v = float(d.var_func(mean))

                    ddvar = d.get_second_deriv_var_func(mean)
                    #print("ddvar = {}".format(ddvar))
                    v += (0.5 * np.trace(ddvar.dot(cov)))

                    dmean = d.get_first_deriv_mean_func(mean)
                    v += (dmean.T.dot(cov.dot(dmean)))

                    vs.append(float(v))

                for i in range(D - D_state):
                    vs.append(0.0)
                #print(vs)

                next_cov += np.diag(vs)
                # TODO add other terms

            z_pred_cov.append(next_cov)

        #print(z_pred_mean)
        x_pred_mean = [z[:D_state] for z in z_pred_mean]
        #print(x_pred_mean)
        x_pred_cov = [s[:D_state, :D_state] for s in z_pred_cov]
        return x_pred_mean, x_pred_cov


def test_pred__1d_dyn_sys():
    # TODO Create disturbance model
    d = GPModel()
    d._params['D'] = 2
    d._params['v0'] = 0.0001
    d._params['v1'] = 0.0001
    d._params['w'] = [0.2, 0.2]
    d.train([np.array([0.65, 0.25])], [0.01])
    # d.train([np.array([0.65, 0.25]), np.array([0.35, 0.25])], [0.01, 0.025])


    # d.train([np.array([1.0, 0.25])], [0.01])
    # d.train([np.array([0.65, 0.25]),
    #          np.array([0.70, 0.25])], [0.01, 0.025])

    fig, (ax1, ax2) = plt.subplots(1, 2)

    # Dynamics
    time_step = 0.1
    A = np.array([1.0])
    B = np.array([time_step])
    dyn = LinearDynamics(A, B, [d])

    x_start = np.array([0.0])

    discrepancy_thresh = 0.05

    num_steps = 25
    # vel = 0.05
    vel = 0.25
    # vel = 0.35
    us = [np.array([vel]) for i in range(num_steps)]
    ts = [time_step * i for i in range(num_steps + 1)]
    xs_no_dist = [x_start]
    for i in range(num_steps):
        x = xs_no_dist[-1]
        x_next = dyn.step(x, us[i], disturbance=False)
        xs_no_dist.append(x_next)

    ax1.set_title("Trajectory ($a = {}$, $b = {}$, $u = {}$ m/sec = const.)".format(float(A), float(B), vel))
    ax1.set_xlabel("$t$ (sec)")
    ax1.set_ylabel("$x$ (m)")

    ax1.set_xlim([0, (num_steps + 1) * time_step])
    ax1.plot(ts, [float(x) for x in xs_no_dist], label="No Disturbance")

    thresh_upper_x = [float(x) + discrepancy_thresh for x in xs_no_dist]
    thresh_lower_x = [float(x) - discrepancy_thresh for x in xs_no_dist]
    ax1.plot(ts, thresh_upper_x, color='blue', linestyle='dashed',
             label="Discrepancy Threshold")
    ax1.plot(ts, thresh_lower_x, color='blue', linestyle='dashed')

    pred_mean, pred_cov = dyn.predict(x_start, us)
    print(len(ts))
    print(len(pred_mean))
    ax1.plot(ts, [float(x) for x in pred_mean],
             color='green', label="Prediction (Mean)")
    pred_std_dev_lower = [float(m) - np.sqrt(float(s))
                      for m, s in zip(pred_mean, pred_cov)]
    pred_std_dev_upper = [float(m) + np.sqrt(float(s))
                      for m, s in zip(pred_mean, pred_cov)]
    ax1.fill_between(
        ts, pred_std_dev_lower, pred_std_dev_upper, color='green', alpha=0.1)

    ax1.legend(loc='upper left')

    ############################################################################
    last_pred_mean = pred_mean[-1]
    last_pred_cov = pred_cov[-1]
    last_desired = xs_no_dist[-1]

    num_x_ends = 1000
    x_ends = np.random.multivariate_normal(
        mean=last_pred_mean[0], cov=last_pred_cov, size=num_x_ends)
    num_x_ends_disc = 0
    for i in range(num_x_ends):
        x_end = x_ends[i]
        err = np.linalg.norm(last_desired - x_end)
        if err > discrepancy_thresh:
            num_x_ends_disc += 1

    print("prob disc = {}".format(1.0 * num_x_ends_disc / num_x_ends))
    ############################################################################

    test_vel = vel
    # test_vel = -0.25
    x_range = [-1.1, 3.1]
    step_size = 0.05
    num_x = int((x_range[1] - x_range[0]) / step_size)
    test_inputs = [np.array([x_range[0] + step_size * i, test_vel])
                   for i in range(num_x)]

    mean = np.array([float(d.mean_func(x)) for x in test_inputs])
    std_dev = np.array([np.sqrt(float(d.var_func(x))) for x in test_inputs])

    ax2.set_title("Disturbance $d(x, u = {}$ m/sec $)$".format(test_vel))
    ax2.set_xlabel("$x$ (m)")
    ax2.set_ylabel("$d$")

    ax2.set_xlim(x_range)
    ax2.set_ylim([-0.1, 0.1])

    ax2.plot([x[0] for x in test_inputs], mean, color='red',
             label="$\mu(x, u = {}$ m/sec $)$".format(test_vel))
    ax2.scatter(
        [x[0] for x in d._training_inputs],
        d._y, marker='+', s=100, c='black', label="Observations")

    ax2.fill_between(
        [x[0] for x in test_inputs],
        mean - std_dev, mean + std_dev, color='red', alpha=0.1,
        label="$\sigma(x, {}$ m/sec $)$".format(test_vel))
    ax2.legend()

    plt.show()


def test_pred__1d_gp():
    inputs = [[1], [3], [4]]
    targets = [0.2, -0.2, 0.8]

    model = GPModel()
    model.train(inputs, targets)

    x_range = [-5.1, 5.1]
    step_size = 0.1
    num_x = int((x_range[1] - x_range[0]) / step_size)
    test_inputs = [[x_range[0] + step_size * i] for i in range(num_x)]

    mean = np.array([float(model.mean_func(x)) for x in test_inputs])
    std_dev = np.array([np.sqrt(float(model.var_func(x))) for x in test_inputs])

    fig, ax = plt.subplots(figsize=(10, 10))

    ax.set_xlim(x_range)
    ax.set_ylim([-1.5, 1.5])

    ax.plot([x[0] for x in test_inputs], mean, label="Test")
    ax.scatter(
        [x[0] for x in inputs],
        targets, marker='+', s=100, c='black', label="Training")

    ax.fill_between(
        [x[0] for x in test_inputs],
        mean - std_dev, mean + std_dev, color='b', alpha=0.1)

    ax.legend()
    plt.show()


def preprocess_training_data(states, controls, errors):
    training_inputs = []
    targets_x = []
    targets_y = []

    for x, u, e in zip(states, controls, errors):
        ok = True
        for z, ex, ey in zip(training_inputs, targets_x, targets_y):
            dist = np.linalg.norm(x - z[:2].flatten())
            #print("x = {}, z = {}, dist = {}".format(x, z[:2].flatten(), dist))
            if dist < 0.035: # TODO Make a parameter
                diff_x = abs(-e[0] - ex)
                diff_y = abs(-e[1] - ey)
                #print("e = {}, diff_x = {}, diff_y = {}".format(e, diff_x, diff_y))
                if diff_x < 0.025 and diff_y < 0.025:
                    ok = False
                    break
        if ok:
            u_normalized = u / np.linalg.norm(u)
            z = np.concatenate([
                x.flatten(), u_normalized.flatten()]).reshape((4, 1))
            training_inputs.append(z)
            targets_x.append(-e[0])
            targets_y.append(-e[1])

    return training_inputs, targets_x, targets_y


def plot_action(ax, dyn, start_state, controls, time_step, discrepancy_thresh):
    ts = [time_step * i for i in range(len(controls) + 1)]
    xs_no_dist = [start_state]
    for i in range(10):
        x = xs_no_dist[-1]
        x_next = dyn.step(x, controls[i], disturbance=False)
        xs_no_dist.append(x_next)

    pred_mean, pred_cov = dyn.predict(start_state, controls)
    print("pred mean = {} cov = {}".format(pred_mean[-1], pred_cov[-1]))

    ########################################################################
    last_pred_mean = pred_mean[-1]
    last_pred_cov = pred_cov[-1]
    last_desired = xs_no_dist[-1]
    print(len(pred_mean))
    print(len(xs_no_dist))

    num_x_ends = 1000
    x_ends = np.random.multivariate_normal(
        mean=last_pred_mean.flatten(), cov=last_pred_cov, size=num_x_ends)
    num_x_ends_disc = 0
    for i in range(num_x_ends):
        x_end = x_ends[i]
        err = np.linalg.norm(last_desired.flatten() - x_end.flatten())
        if err > discrepancy_thresh:
            num_x_ends_disc += 1

    prob_disc = 1.0 * num_x_ends_disc / num_x_ends
    print("prob disc = {}".format(prob_disc))
    ax.text(last_desired[0], last_desired[1], str(prob_disc), color='red')
    ########################################################################

    # Plot the trajectory w/o disturbances.
    ax.plot([x[0] for x in xs_no_dist],
            [x[1] for x in xs_no_dist],
            color='red', linewidth=2)
    e = Ellipse(xy=(xs_no_dist[-1][0], xs_no_dist[-1][1]),
                width=discrepancy_thresh,
                height=discrepancy_thresh,
                fill=False, edgecolor='red', linestyle='dashed')
    ax.add_artist(e)

    ax.plot([x[0] for x in pred_mean],
            [x[1] for x in pred_mean],
            color='orange', linewidth=2)
    for mean, cov in zip(pred_mean, pred_cov):
        e = Ellipse(xy=(mean[0], mean[1]),
                    width=2.0 * np.sqrt(cov[0, 0]),
                    height=2.0 * np.sqrt(cov[1, 1]),
                    fill=True, facecolor='orange', alpha=0.2,
                    edgecolor='orange')
        ax.add_artist(e)


def from_bagfile(bagfile):
    thresh = 0.015 # Discrepancy threshold
    max_speed = 0.1 # m/s

    discrepancy_thresh = 0.075

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
            time_step = (msg.time[i + 1] - msg.time[i]).to_sec()

            if time_step < 1e-3 or time_step > 1.0:
                continue

            pos = np.array([actual_pose.position.x,
                            actual_pose.position.y])
            desired_pos = np.array([desired_pose.position.x,
                                    desired_pose.position.y])
            to_desired = desired_pos - pos

            ctl_vel = to_desired / time_step
            ctl_speed = np.linalg.norm(ctl_vel)
            if ctl_speed > max_speed:
                # Enforce the max speed.
                ctl_vel = max_speed * ctl_vel / ctl_speed

            expected_next_pos = pos + time_step * ctl_vel
            actual_next_pos = np.array([next_actual_pose.position.x,
                                        next_actual_pose.position.y])
            error = expected_next_pos - actual_next_pos

            if np.linalg.norm(error) > thresh:
                states.append(pos)
                controls.append(ctl_vel)
                errors.append(error)
                print("x: {}, u: {}, e: {}, ||e||: {}, dt: {}".format(
                    states[-1], controls[-1], errors[-1],
                    np.linalg.norm(errors[-1]), time_step))

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.scatter([p.position.x for p in msg.desired_pose],
                   [p.position.y for p in msg.desired_pose],
                   marker='+', s=100, c='black', label="Desired")
        ax.scatter([p.position.x for p in msg.actual_pose],
                   [p.position.y for p in msg.actual_pose],
                   marker='o', s=100, c='blue', alpha=0.2, label="Actual")
        ax.quiver([s[0] for s in states],
                  [s[1] for s in states],
                  [-e[0] for e in errors],
                  [-e[1] for e in errors], color='red', alpha=0.2, label="Disturbance")


        # normalized_controls = [u / np.linalg.norm(u) for u in controls]
        # print("normalized_ctls = {}".format(normalized_controls))

        print("Before preprocessing: num states: {}".format(len(states)))
        training_inputs, targets_x, targets_y = \
            preprocess_training_data(states, controls, errors)
        print("After preprocessing: num inputs: {}".format(len(training_inputs)))

        ax.scatter([z[0] for z in training_inputs],
                   [z[1] for z in training_inputs],
                   marker='+', s=150, c='green', label="Training Input ($x$)")
        u_scale = 0.1
        ax.quiver([z[0] for z in training_inputs],
                  [z[1] for z in training_inputs],
                  [u_scale * z[2] for z in training_inputs],
                  [u_scale * z[3] for z in training_inputs],
                  color='green', alpha=0.5, label="Training Input ($\hat{u}$)")

        # ax.legend()
        # plt.show()

        # Model of disturbance along the x-position.
        disturbance_x = GPModel()
        disturbance_x._params['D'] = 4
        disturbance_x._params['v0'] = 1e-4
        disturbance_x._params['v1'] = 1e-4 # 5e-4 # 0.1 # 1.0 # 1e-4
        disturbance_x._params['w'] = [0.25, 0.25, 0.75, 0.75] # [0.4, 0.4, 0.4, 0.4]
        disturbance_x.train(training_inputs, targets_x)

        # Model of disturbance along the y-position.
        disturbance_y = GPModel()
        disturbance_y._params['D'] = 4
        disturbance_y._params['v0'] = 1e-4
        disturbance_y._params['v1'] = 1e-4 #5e-4 # 0.1 # 1.0 # 1e-4
        disturbance_y._params['w'] = [0.25, 0.25, 0.75, 0.75] # [0.4, 0.4, 0.4, 0.4]
        disturbance_y.train(training_inputs, targets_y)

        # Dynamics.
        time_step = 0.1
        A = np.eye(2)
        B = time_step * np.eye(2)
        dyn = LinearDynamics(A, B, [disturbance_x, disturbance_y])

        # TODO Fwd sim different actions
        # start_state = np.array([-0.2, 0.35]).reshape((2, 1))
        # start_state = np.array([-0.3, 0.35]).reshape((2, 1))
        # start_state = np.array([-0.4, 0.35]).reshape((2, 1))
        # start_state = np.array([-0.2, 0.35]).reshape((2, 1))
        # start_state = np.array([-0.1, 0.30]).reshape((2, 1))
        # start_state = np.array([-0.1, 0.25]).reshape((2, 1))
        start_state = np.array([0, 0.2]).reshape((2, 1))
        controls_move_pos_x = [np.array([0.1, 0.0]).reshape((2, 1))
                               for i in range(10)]
        controls_move_neg_x = [np.array([-0.1, 0.0]).reshape((2, 1))
                               for i in range(10)]
        controls_move_pos_y = [np.array([0.0, 0.1]).reshape((2, 1))
                               for i in range(10)]
        controls_move_neg_y = [np.array([0.0, -0.1]).reshape((2, 1))
                               for i in range(10)]

        actions = [controls_move_pos_x,
                   controls_move_neg_x,
                   controls_move_pos_y,
                   controls_move_neg_y]
        for a in actions:
            plot_action(
                ax, dyn, start_state, a, time_step, discrepancy_thresh)

        # controls_move_pos_x = [np.array([0.0, -0.1]).reshape((2, 1))
        #                        for i in range(10)]
        # ts = [time_step * i for i in range(11)]
        # xs_no_dist = [x_start]
        # for i in range(10):
        #     x = xs_no_dist[-1]
        #     x_next = dyn.step(x, controls_move_pos_x[i], disturbance=False)
        #     xs_no_dist.append(x_next)

        # pred_mean, pred_cov = dyn.predict(x_start, controls_move_pos_x)
        # print("pred mean = {} cov = {}".format(pred_mean[-1], pred_cov[-1]))

        # ########################################################################
        # last_pred_mean = pred_mean[-1]
        # last_pred_cov = pred_cov[-1]
        # last_desired = xs_no_dist[-1]
        # print(len(pred_mean))
        # print(len(xs_no_dist))

        # num_x_ends = 1000
        # x_ends = np.random.multivariate_normal(
        #     mean=last_pred_mean.flatten(), cov=last_pred_cov, size=num_x_ends)
        # num_x_ends_disc = 0
        # for i in range(num_x_ends):
        #     x_end = x_ends[i]
        #     err = np.linalg.norm(last_desired.flatten() - x_end.flatten())
        #     if err > discrepancy_thresh:
        #         num_x_ends_disc += 1

        # print("prob disc = {}".format(1.0 * num_x_ends_disc / num_x_ends))
        ########################################################################

        # # Plot the trajectory w/o disturbances.
        # ax.plot([x[0] for x in xs_no_dist],
        #         [x[1] for x in xs_no_dist],
        #         color='red', linewidth=2)
        # e = Ellipse(xy=(xs_no_dist[-1][0], xs_no_dist[-1][1]),
        #             width=discrepancy_thresh,
        #             height=discrepancy_thresh,
        #             fill=False, edgecolor='red', linestyle='dashed')
        # ax.add_artist(e)

        # thresh_1 = []
        # thresh_2 = []
        # for i in range(len(xs_no_dist) - 1):
        #     r = xs_no_dist[i + 1] - xs_no_dist[i]
        #     r /= np.linalg.norm(r)

        #     r_perp_1 = np.array([-r[1], r[0]])
        #     r_perp_2 = np.array([r[1], -r[0]])

        #     thresh_1.append(xs_no_dist[i] + discrepancy_thresh * r_perp_1)
        #     thresh_2.append(xs_no_dist[i] + discrepancy_thresh * r_perp_2)

        # ax.plot([x[0] for x in thresh_1],
        #         [x[1] for x in thresh_1],
        #         color='red', linestyle='dashed')
        # ax.plot([x[0] for x in thresh_2],
        #         [x[1] for x in thresh_2],
        #         color='red', linestyle='dashed')

        # ax.plot([x[0] for x in pred_mean],
        #         [x[1] for x in pred_mean],
        #         color='orange', linewidth=2)
        # for mean, cov in zip(pred_mean, pred_cov):
        #     e = Ellipse(xy=(mean[0], mean[1]),
        #                 width=2.0 * np.sqrt(cov[0, 0]),
        #                 height=2.0 * np.sqrt(cov[1, 1]),
        #                 fill=True, facecolor='orange', alpha=0.2,
        #                 edgecolor='orange')
        #     # e.set_facecolor('none')
        #     ax.add_artist(e)
        # TODO Plot std dev of prediction (how to do?)
        ax.legend()
        plt.show()

        # training_inputs = [np.concatenate([x, u])
        #                    for x, u in zip(states, normalized_controls)]
        # targets_x = [-e[0] for e in errors]
        # targets_y = [-e[1] for e in errors]

        # dist_x_model = GP(
        #     training_inputs, targets_x, noise_std_dev=0.1)
        # dist_y_model = GP(
        #     training_inputs, targets_y, noise_std_dev=0.1)

        ########################################################################
        # test_inputs = []

        # # px_range = [-0.2, 0.0]
        # # py_range = [0.2, 0.35]

        # # px_range = [-0.4, 0.4]
        # # py_range = [-0.4, 0.4]

        # px_range = [-0.2, 0]
        # py_range = [-0.2, 0.4]

        # # step_size = 0.05
        # step_size = 0.01
        # num_px = int((px_range[1] - px_range[0]) / step_size)
        # num_py = int((py_range[1] - py_range[0]) / step_size)
        # for i in range(num_px):
        #     px = px_range[0] + step_size * i
        #     for j in range(num_py):
        #         py = py_range[0] + step_size * j
        #         z = np.array([px, py, 0.0, 1.0])
        #         test_inputs.append(z)

        # mean, cov = dist_y_model.predict(test_inputs)

        # fig = plt.figure()
        # ax = plt.axes(projection='3d')
        # ax.scatter3D(
        #     [z[0] for z in test_inputs],
        #     [z[1] for z in test_inputs],
        #     mean, c=mean, cmap='Greens')
        # ax.scatter3D([z[0] for z in training_inputs],
        #              [z[1] for z in training_inputs],
        #              [-e[1] for e in errors], c='red')
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')
        # ax.set_zlabel('z')
        # plt.show()
        ########################################################################


if __name__ == '__main__':
    # test_pred__1d_dyn_sys()
    # test_pred__1d_gp()

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    from_bagfile(args.bag)
