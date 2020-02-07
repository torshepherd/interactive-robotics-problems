from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

DISTANCE_COST = 1
ANGLE_COST = 1
GOAL_COST = 1


def generate_maze(pixel_dimensions, sparsity):
    map_array = np.zeros(pixel_dimensions)
    num_tiles = int(pixel_dimensions[0] * pixel_dimensions[1] * sparsity)
    for i in range(num_tiles):
        map_array[np.random.randint(0, pixel_dimensions[0]), np.random.randint(
            0, pixel_dimensions[1])] = 1
    return map_array


def random_sample_with_obstacles(maze, num_samples):
    distribution = np.random.rand(
        num_samples, 3) * np.array([[np.shape(maze)[0], np.shape(maze)[1], 2 * np.pi]])
    filtered_dist = np.empty((0, 3), float)
    for point in np.copy(distribution):
        map_index = (int(point[0]//1), int(point[1]//1))
        if maze[map_index] == 0:
            filtered_dist = np.append(filtered_dist, [point], 0)
    return filtered_dist


main_maze = generate_maze((10, 10), 0.2)
dist = random_sample_with_obstacles(main_maze, 1000)
print(main_maze)
x, y, z = dist.T
goal_pose = np.array([10, 10, 0])


def cost_fun(indices_of_points):
    points = dist[np.array(indices_of_points)]
    difference_array = np.append(
        np.array([[0, 0, 0]]), 
        points[1:] - points[:-1], 0)
    distance_array_0 = np.sqrt(
        (difference_array[:, 0] ** 2 + difference_array[:, 1] ** 2))
    distance_array = np.array(
        [distance_array_0, 
         np.abs(difference_array[:, 2] % (2 * np.pi))]).T

    cost = np.sum(distance_array * np.array([[DISTANCE_COST, ANGLE_COST]])) + np.sum((goal_pose - points) * GOAL_COST)

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.scatter(x, y, z)
plt.show()
