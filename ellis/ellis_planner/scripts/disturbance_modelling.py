import argparse
import rosbag
import matplotlib.pyplot as plt
import matplotlib as mpl
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
            #print("A = {}, mean = {}, A.dot(mean) = {}, d = {}".format(A, mean, tmp, d))
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
                    v += (0.5 * np.trace(ddvar.dot(cov)))

                    dmean = d.get_first_deriv_mean_func(mean)
                    v += (dmean.T.dot(cov.dot(dmean)))

                    vs.append(float(v))

                next_cov += np.diag(vs)
                # TODO add other terms

            z_pred_cov.append(next_cov)

        #print(z_pred_mean)
        x_pred_mean = [z[:D_state] for z in z_pred_mean]
        #print(x_pred_mean)
        x_pred_cov = [s[:D_state, :D_state] for s in z_pred_cov]
        return x_pred_mean, x_pred_cov



    def predict_v0(self, x_start, us):
        D_state = len(x_start)

        x_pred_mean = [x_start]
        x_pred_cov = [np.zeros((D_state, D_state))]

        for u in us:
            prev_mean = x_pred_mean[-1]
            prev_cov = x_pred_cov[-1]

            print("mean = {}, cov = {}".format(prev_mean, prev_cov))

            # Predict the mean.
            z = np.concatenate([prev_mean.ravel(), u.ravel()])
            d_mean = np.array([d.mean_func(z)
                               for d in self._d]).reshape((D_state, 1))
            next_mean = self._A.dot(prev_mean) + self._B.dot(u) + d_mean
            x_pred_mean.append(next_mean)

            # Predict the covariance.
            next_cov = np.zeros((D_state, D_state))
            # next_cov = 1e-2 * np.ones((D_state, D_state))
            if len(x_pred_cov) == 1:
                next_cov = np.diag([float(d.var_func(z))
                                    for d in self._d])
            else:
                next_cov = self._A.dot(prev_cov.dot(self._A.T)).reshape(
                    (D_state, D_state))

                vs = []
                for d in self._d:
                    v = d.var_func(z)

                    ddvar = d.get_second_deriv_var_func(z)
                    v += (0.5 * np.trace(ddvar.dot(prev_cov)))

                    dmean = d.get_first_deriv_mean_func(z)
                    v += (dmean.T.dot(prev_cov.dot(dmean)))

                    vs.append(float(v))

                next_cov += np.diag(vs)
                # TODO add other terms

            x_pred_cov.append(next_cov)

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


class GP:
    def __init__(self, inputs=[], targets=[], noise_std_dev=0.01):
        self._inputs = inputs
        self._targets = targets
        self._noise_std_dev = noise_std_dev

    def cov(self, xp, xq, l=1.0, signal_std_dev=1.0):
        norm = np.linalg.norm(xp - xq)
        return (signal_std_dev**2 * np.exp(-0.5 * norm**2 / l**2))

    def predict(self, test_inputs):
        n_train = len(self._inputs)
        n_test = len(test_inputs)

        # Construct covariance matrices
        K_train = np.zeros((n_train, n_train))
        for i in range(n_train):
            for j in range(n_train):
                K_train[i, j] = self.cov(self._inputs[i], self._inputs[j])

        K_test = np.zeros((n_test, n_test))
        for i in range(n_test):
            for j in range(n_test):
                K_test[i, j] = self.cov(test_inputs[i], test_inputs[j])

        K_train_test = np.zeros((n_train, n_test))
        for i in range(n_train):
            for j in range(n_test):
                K_train_test[i, j] = self.cov(self._inputs[i], test_inputs[j])

        A = K_train + self._noise_std_dev**2 * np.eye(n_train)
        # print("e-vals of A: {}".format(np.linalg.eigvals(A)))
        # Define Z = inv(A) * transpose(K_test_train)
        #         ==> AZ = transpose(K_test_train)
        Z = scipy.linalg.solve(A, K_train_test, sym_pos=True).T

        y = np.array(self._targets)

        mean = Z.dot(y)
        cov = K_test - np.dot(Z, K_train_test)
        # print("e-vals of cov: {}".format(np.linalg.eigvals(cov)))

        return mean, cov


def test_gp_1d():
    # inputs = [1, 3, 4]
    # targets = [0.2, -0.2, 0.8]

    inputs = [0, 0.05, 0.1, 0.15, 0.2, 3.0]
    targets = [0.5, 0.55, 0.48, 0.58, 0.52, -0.5]

    model = GP(inputs, targets, noise_std_dev=0.05)

    x_range = [-5.1, 5.1]
    step_size = 0.1
    num_x = int((x_range[1] - x_range[0]) / step_size)
    test_inputs = [x_range[0] + step_size * i for i in range(num_x)]

    mean, cov = model.predict(test_inputs)

    fig, ax = plt.subplots(figsize=(10, 10))

    ax.set_xlim(x_range)
    ax.set_ylim([-1.5, 1.5])

    ax.plot(test_inputs, mean, label="Test")
    ax.scatter(inputs, targets, marker='+', s=100, c='black', label="Training")

    std_dev = np.sqrt(np.diag(cov))
    ax.fill_between(test_inputs, mean - std_dev, mean + std_dev, color='b', alpha=0.1)

    test_inputs = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5]
    mean, cov = model.predict(test_inputs)
    y_test = np.random.multivariate_normal(
        mean=mean, cov=cov, size=1)
    ax.scatter(test_inputs, y_test, marker='o', c='blue')

    ax.legend()
    plt.show()


def from_bagfile(bagfile):
    thresh = 0.015 # Discrepancy threshold
    max_speed = 0.1 # m/s

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
                   marker='o', s=100, c='blue', label="Actual")
        ax.quiver([s[0] for s in states],
                  [s[1] for s in states],
                  [-e[0] for e in errors],
                  [-e[1] for e in errors], color='red', label="Disturbance")

        ax.legend()
        plt.show()

        normalized_controls = [u / np.linalg.norm(u) for u in controls]
        print("normalized_ctls = {}".format(normalized_controls))

        training_inputs = [np.concatenate([x, u])
                           for x, u in zip(states, normalized_controls)]
        targets_x = [-e[0] for e in errors]
        targets_y = [-e[1] for e in errors]

        dist_x_model = GP(
            training_inputs, targets_x, noise_std_dev=0.1)
        dist_y_model = GP(
            training_inputs, targets_y, noise_std_dev=0.1)

        sim_end_pos = fwd_sim_OL(
            np.array([10, 10]),            # start
            np.array([10.1, 10]),          # end
            [dist_x_model, dist_y_model],
        )
        print(sim_end_pos)

        sim_end_pos = fwd_sim_OL(
            np.array([0, 0]),            # start
            np.array([0.1, 0]),          # end
            [dist_x_model, dist_y_model],
        )
        print(sim_end_pos)

        sim_end_pos = fwd_sim_OL(
            np.array([-0.25, 0.35]),      # start
            np.array([-0.15, 0.35]),      # end
            [dist_x_model, dist_y_model],
        )
        print(sim_end_pos)

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


def fwd_sim_OL(start_pos, end_pos, dist_model, max_speed=0.1, time_step=0.2):
    ctls = []

    # Generate (open-loop) sequence of controls.
    dist = np.linalg.norm(end_pos - start_pos)
    step_size = max_speed * time_step
    num_steps = int(dist / step_size) + 1
    pos = start_pos
    for i in range(num_steps):
        to_end = end_pos - pos
        dist = np.linalg.norm(to_end)
        u = np.array([0, 0])
        if dist < step_size:
            u = to_end
        else:
            u = max_speed * to_end / dist
        ctls.append(u)

    thresh = 0.05
    grid_size = 0.01
    u_dir = (end_pos - start_pos) / np.linalg.norm(end_pos - start_pos)
    print("u_dir = {}".format(u_dir))
    px_range = [start_pos[0] - thresh, end_pos[0] + thresh]
    py_range = [start_pos[1] - thresh, end_pos[1] + thresh]
    num_px = int((px_range[1] - px_range[0]) / grid_size)
    num_py = int((py_range[1] - py_range[0]) / grid_size)
    grid = num_px * [num_py * [(0.0, 0.0)]]

    test_inputs = []
    for i in range(num_px):
        px = px_range[0] + i * grid_size
        for j in range(num_py):
            py = py_range[0] + j * grid_size
            z = np.array([px, py, u_dir[0], u_dir[1]])
            test_inputs.append(z)
    mean_x, cov_x = dist_model[0].predict(test_inputs)
    cov_x += (1e-6 * np.eye(len(test_inputs)))
    dx = np.random.multivariate_normal(
        mean=mean_x, cov=cov_x, size=1).flatten()
    mean_y, cov_y = dist_model[1].predict(test_inputs)
    cov_y += (1e-6 * np.eye(len(test_inputs)))
    dy = np.random.multivariate_normal(
        mean=mean_y, cov=cov_y, size=1).flatten()

    ############################################################################
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.quiver([z[0] for z in test_inputs],
              [z[1] for z in test_inputs],
              dx, dy, color='red', label="Disturbance")
    ax.legend()
    plt.show()
    ############################################################################

    for i in range(num_px):
        for j in range(num_py):
            idx = i * num_py + j
            grid[i][j] = (dx[idx], dy[idx])

    # Forward simulate.
    pos = start_pos
    for u in ctls:
        print("fwd sim x: {}, u: {}".format(pos, u))
        # Get the sampled disturbance.
        i = int((pos[0] - px_range[0]) / grid_size)
        j = int((pos[1] - py_range[0]) / grid_size)
        d = np.array([0, 0])
        if i >= 0 and i < num_px and j >= 0 and j < num_py:
            d = np.array(grid[i][j])
        print("  d = {}".format(d))

        pos = pos + time_step * u + d

    return pos

if __name__ == '__main__':
    test_pred__1d_dyn_sys()
    # test_pred__1d_gp()
    # test_gp_1d()

    # parser = argparse.ArgumentParser()
    # parser.add_argument("bag")
    # args = parser.parse_args()

    # from_bagfile(args.bag)
