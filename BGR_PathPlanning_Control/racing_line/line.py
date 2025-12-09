import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Todo: create a config file for parameters like epsilon, number of points for smoothing, etc.

class Line:
    """
    Simple racing-line container. Keeps track of all points visited during a lap and
    merges new stages by snapping them to the closest existing point.
    """

    def __init__(self, initial_position=(0.0, 0.0), epsilon=1.2):
        """
        Args:
            initial_position: tuple-like (x, y) starting position that should
                              always be part of the racing line.
            epsilon: maximum distance (m) to consider two points the same anchor.
        """
        self.epsilon = float(epsilon)   # epsilon distance for matching points
        self.last_index = 0             # last matched index to speed up search
        self.path = np.asarray(initial_position, dtype=float).reshape(1, 2) # reshape - ensure 2D array with one point
        self.finished = False           # whether the lap is finished

    def add_stage(self, new_stage):
        """
        Merge a new array of (x, y) points into the existing path.
        """
        if new_stage is None or self.finished:
            return

        stage = np.asarray(new_stage, dtype=float) # ensure numpy array
        if stage.size == 0:
            return

        if stage.ndim != 2 or stage.shape[1] != 2:
            raise ValueError("Each point must contain exactly two coordinates (x, y)")

        start_point = stage[0]
        closest_index = self.__closest_point_index(self.path, start_point)

        if closest_index != -1:
            self.path = np.vstack((self.path[:closest_index], stage))
    
    def get_path(self):
        """
        Returns a copy of the completed racing line.
        """
        return self.path.copy()
    
    def set_path(self, new_path: np.ndarray):
        """
        Set the racing line to a new path.
        deep copy is made to avoid external modifications.
        :param new_path: ndarray of shape (N, 2) representing the new path.
        """
        if new_path is not None and new_path.ndim == 2 and new_path.shape[1] == 2:
            self.path = np.asarray(new_path, dtype=float).copy()

    def set_finished(self, finished=True):
        '''
        Set the finished state of the lap.
        '''
        self.finished = finished

    def did_end_lap(self,car_position):
        """
        Check if the given point is close enough to the starting point to consider the lap finished.
        """
        if(len(self.path) == 1):
            return False
        
        start_point = self.path[0]
        distance_to_start = self.__distance(start_point, car_position)
        # return distance_to_start < self.epsilon
        return distance_to_start < 2 # 2 meters for testing purposes

    def smooth_path(self, path: np.ndarray = None):
        """
        Smooth the racing line using cubic splines.
        assume the path is a closed loop, so the first and last points are the same.
        
        :param self:
        :param path: ndarray of shape (N, 2) representing the path to be smoothed. If None, uses self.path.
        :type path: np.ndarray
        """
        path = self.path if path is None else path

        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[1] != 2:
            raise ValueError("Path must be a 2D array with shape (N, 2)")

        # make 1D array of x and y
        x = path[:, 0]
        y = path[:, 1]
        t = np.linspace(0, 1, len(path), endpoint=False) # use evenly spaced number between 0 and 1 instead of the indexes of the points - 1. t ∈ [0,1] better for closed loops, 2. Integration and derivatives become nicer - good for curvature, velocity and other calculations, 3. it behaves better with other SciPy functions(they assume t ∈ [0,1])

        cs_x = CubicSpline(t, x, bc_type='periodic') # periodic means the first and last point are connected
        cs_y = CubicSpline(t, y, bc_type='periodic')

        t_new = np.linspace(0, 1, 200) # TODO : 1. choose the best number of points; 2. make it a parameter(maybe in a config file)
        x_smooth = cs_x(t_new)
        y_smooth = cs_y(t_new)

        smoothed_path = np.vstack((x_smooth, y_smooth)).T # shape (N, 2), creates (2, N) array and then transposes to (N, 2)
        return smoothed_path

    # -------------------------- utility functions ----------------------------- #
    def __distance(self, point1, point2):
        delta = point1 - point2
        return np.hypot(delta[0], delta[1])

    def __closest_point_index(self, path, point):
        """
        Find the index of the point in `path` that is within epsilon of the given point.
        Returns -1 if nothing is close enough.
        """
        if path.size == 0:
            return -1
        min_dist = float('inf')
        closest_index = -1
        for i in range(self.last_index, len(path)):
            # if self.__distance(path[i], point) < self.epsilon:
            #     return i
            # return -1
            if self.__distance(path[i], point) < min_dist:
                min_dist = self.__distance(path[i], point)
                closest_index = i   
            self.last_index = closest_index
        return closest_index

    def cut_end(self):
        '''
        The function will look for the closest point to the start point in the path(from index) 
        and cut the rest of the path after that point.
        '''
        start_point = self.path[0]
        closest_index = self.__closest_point_index(self.path, start_point)
        if closest_index != -1:
            self.path = self.path[:closest_index]
            self.path = np.vstack((self.path, start_point)) # add start point to the end to close the loop

def main():
    """
    Basic smoke test for the line class: adds two stages and prints the resulting path.
    """
    racing_line = Line(initial_position=(0.0, 0.0), epsilon=0.75)

    stage_one = np.array(
        [
            [0.0, 0.0],
            [5.0, 1.0],
            [10.0, 2.0],
        ]
    )
    stage_two = np.array(
        [
            [5.1, 1.2],
            [10.1, 2.1],
            [12.0, 4.5],
            [15.0, 6.0],
        ]
    )

    stage_three = np.array(
        []
    )

    stage_four = np.array(
        [
            [0.0, 0.0],
        ]
    )

    racing_line.add_stage(stage_one)
    racing_line.add_stage(stage_two)
    racing_line.add_stage(stage_three)
    racing_line.add_stage(stage_four)

    result = racing_line.get_path()
    print("Resulting racing line points:")
    print(result)

    fig, ax = plt.subplots()
    ax.plot(result[:, 0], result[:, 1], "k.-", linewidth=2, label="Combined line")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Racing line (combined)")
    ax.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
