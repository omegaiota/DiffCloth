import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits import mplot3d
import math
from pathlib import Path
from colorama import Fore, Style

def plotPointCloud(pointMat): # should be   #numPoints x 3
    if (pointMat.shape[1] != 3):
        print("Trying to plot point cloud. expect matrix size to be (#numPoints, 3), but instead get {}".format(pointMat.shape))
    x = pointMat[:, 0]
    y = pointMat[:, 1]
    z = pointMat[:, 2]

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z)
    plt.show()


def plotPointCloudFromArr(x_vec):
    x, y, z = [], [], []
    for i in range(len(x_vec)):
        p = x_vec[i]
        x.append(p[0])
        y.append(p[1])
        z.append(p[2])

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X', fontsize=20)
    ax.set_ylabel('Z')
    ax.set_zlabel('Y')
    ax.scatter(x, z, y)
    plt.show()


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def plotPointCloudFromVec(x_vec):
    x, y, z = [], [], []
    mean = [0,0,0]
    for i in range(x_vec.shape[0] // 3):
        p = x_vec[i*3:i*3+3]
        x.append(p[0])

        y.append(p[2])
        z.append(p[1])
        for dim in range(3):
            mean[dim] = mean[dim] + p[dim]

    for dim in range(3):
        mean[dim] = mean[dim] / len(x)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z)



    ax.view_init(elev=10., azim=10)
    ax.scatter([mean[0]],[mean[1]],[mean[2]], 'r')
    set_axes_equal(ax)
    plt.show()


def plotPointCloudFromVecs(vecs, myTitle = '', save = False, path = ''):

    fig = plt.figure(figsize=(16, 8))
    colors = ['r', 'g', 'b']

    xyzs = []
    for (id, x_vec) in enumerate(vecs):
        x, y, z = [], [], []
        for i in range(x_vec.shape[0] // 3):
            p = x_vec[i*3:i*3+3]
            x.append(p[0])

            y.append(p[2])
            z.append(p[1])
        xyzs.append((x,y,z))


    views = [(10, 110), (90, 110)]
    for (viewid, (angle1, angle2)) in enumerate(views):
        ax = fig.add_subplot(121 + viewid, projection='3d')
        ax.view_init(elev=angle1, azim=angle2)
        for (id, _) in enumerate(vecs):
            (x,y,z) = xyzs[id]
            ax.scatter(x, y, z, colors[id % len(colors)])
        set_axes_equal(ax)

    plt.title(myTitle)
    if save:
        plt.savefig(  str(path) +  '-v3.png', bbox_inches='tight', pad_inches=0 )
    else:
        plt.show()

def getPointOnSphere(radius, phi, theta): # phi: changes x,z  theta: changes y (up and down)
    p = radius * np.array([math.cos(phi) * math.sin(theta), math.cos(theta), math.sin(phi) * math.sin(theta)])
    return p


def plotLosses(trainLosses, evalLosses, path):
    epochs = range(1, len(trainLosses)+1)

    fig = plt.figure()#figsize=(3, 6)

    plt.plot(epochs, trainLosses, 'r', label='Training loss')
    plt.plot(epochs, evalLosses, 'b', label='Validation loss')
    plt.title('Training and Validation loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()
    plt.savefig(  "{}/losses.png".format(path))


def plotCurve(data, name, path):
    epochs = range(1, len(data)+1)

    fig = plt.figure()#figsize=(3, 6)

    plt.plot(epochs, data, 'r', label=name)
    plt.title(name)
    plt.xlabel('Epochs')
    plt.ylabel(name)
    plt.legend()
    plt.savefig(  "{}/{}.png".format(path, name))

def log(str, writeToLog = False, logFile = None):
    print("{}{}{}".format(Fore.GREEN, str, Style.RESET_ALL) )
    if writeToLog:
        logFile.write(str)
