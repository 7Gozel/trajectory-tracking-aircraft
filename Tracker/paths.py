import numpy as np
import matplotlib.pyplot as plt


def generate_path(p_case):
    t = np.arange(0, 2500)

    if p_case == 1:
        path = np.zeros((len(t), 3))
        path[:, 0] = 4 * t
        path[:, 1] = 6 * t
        path[:, 2] = 0.4 * t
    elif p_case == 2:
        omp = 0.0025
        path1 = 6000 * np.column_stack((np.sin(omp * t), np.cos(omp * t), 0.1 * t))
        path1 -= path1[0,:]

        path1[:, 2] = 0.4 * t
        path2 = np.zeros((len(t), 3))
        path3 = np.zeros((len(t), 3))
        path4 = np.zeros((len(t), 3))
        path2 += path1[-1, :]
        path2[:, 0] += 6 * t
        path2[:, 2] += 0.4 * t
        path3 += path2[-1, :]
        path3[:, 0] += 3 * t
        path4 += path3[-1, :]
        path4[:, 0] += 4 * t
        path4[:, 2] -= 0.4 * t
        path = np.concatenate((path1, path2, path3, path4), axis=0)
    elif p_case == 3:
        omp = 0.0014
        path = np.zeros((len(t), 3))
        path[:, 0] = 15 * t
        path[:, 1] = 200 * (-np.cos(2 * np.pi * omp * t) + 1) * np.exp(0.002 * t)
        path[:, 2] = 0.5 * t
    elif p_case == 4:
        omp = 0.001
        path = np.zeros((len(t), 3))
        path[:, 0] = 10 * t
        path[:, 1] = 15000 * np.sin(2 * np.pi * omp * t)
        path[:, 2] = 0.3 * t
    elif p_case == 5:
        omp = 0.001
        path = np.zeros((len(t), 3))
        path[:, 0] = 4000 * np.cos(2 * np.pi * omp * t)
        path[:, 1] = 6000 * np.sin(2 * np.pi * omp * t)
        path[:, 2] = 0.4 * t
    elif p_case == 6:
        omp = 0.001
        t1 = t[:len(t) // 2]
        t2 = t[len(t) // 2:]
        path1 = np.zeros((len(t1), 3))
        path2 = np.zeros((len(t2), 3))
        path3 = np.zeros((len(t2), 3))
        path1[:, 0] = -4000 * np.cos(2 * np.pi * omp * t1)+4000
        path1[:, 1] = 6000 * np.sin(2 * np.pi * omp * t1)
        path1[:, 2] = 0.4 * t1
        path2[:, 0] = 3 * t1 + path1[-1, 0]
        path2[:, 1] = 5 * t1 + path1[-1, 1]
        path2[:, 2] = 0.4 * t2
        path3[:, 0] = 4000 * np.cos(2 * np.pi * omp * t1) - 4000 + path2[-1, 0]
        path3[:, 1] = 6000 * np.sin(2 * np.pi * omp * t1) + path2[-1, 1]
        path3[:, 2] = 0.4 * t1 + path2[-1, 2]
        path = np.concatenate((path1, path2, path3), axis=0)

    return path


if __name__ == "__main__":
    p_case_test = 3
    generated_path = generate_path(p_case_test)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.grid(True)
    ax.scatter(generated_path[:, 0], generated_path[:, 1], generated_path[:, 2], label='Path')

    ax.legend()
    ax.grid(True)

    plt.show()
