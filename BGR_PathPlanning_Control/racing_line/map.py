import numpy as np
import matplotlib.pyplot as plt


class line:
    """
    Simple racing-line container. Keeps track of all points visited during a lap and
    merges new stages by snapping them to the closest existing point.
    """

    def __init__(self, initial_position=(0.0, 0.0), epsilon=0.5):
        """
        Args:
            initial_position: tuple-like (x, y) starting position that should
                              always be part of the racing line.
            epsilon: maximum distance (m) to consider two points the same anchor.
        """
        self.epsilon = float(epsilon)
        self.last_index = 0
        self.path = np.asarray(initial_position, dtype=float).reshape(1, 2) # reshape - ensure 2D array with one point

    def add_stage(self, new_stage):
        """
        Merge a new array of (x, y) points into the existing path.
        """
        if new_stage is None:
            return

        stage = np.asarray(new_stage, dtype=float) # ensure numpy array
        if stage.size == 0:
            return

        if stage.ndim == 1: 
            stage = stage.reshape(1, -1) # -1 means infer size

        if stage.shape[1] != 2:
            raise ValueError("Each point must contain exactly two coordinates (x, y)")

        start_point = stage[0]
        closest_index = self.__closest_point_index(self.path, start_point)

        if closest_index != -1:
            self.path = np.vstack((self.path[:closest_index + 1], stage))

    def end_lap(self):
        """
        Returns a copy of the completed racing line.
        """
        return self.path.copy()

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

        for i in range(self.last_index, len(path)):
            if self.__distance(path[i], point) < self.epsilon:
                self.last_index = i
                return i
        return -1


def main():
    """
    Basic smoke test for the line class: adds two stages and prints the resulting path.
    """
    racing_line = line(initial_position=(0.0, 0.0), epsilon=0.75)

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

    result = racing_line.end_lap()
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
