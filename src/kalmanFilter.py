import numpy as np
from filterpy.kalman import KalmanFilter


# Create a linear Kalman Filter for a 2D system prediction
# state = [x, y, x_dot, y_dot, x_dot_dot, y_dot_dot]
def create_kalman_filter(dt):
    kf = KalmanFilter(dim_x=6, dim_z=2)

    # State transition matrix
    kf.F = np.array(
        [
            [1, 0, dt, 0, 0.5 * dt**2, 0],
            [0, 1, 0, dt, 0, 0.5 * dt**2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ]
    )

    # Measurement matrix
    kf.H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])

    # Covariance matrix
    kf.P *= 100

    # Process noise covariance matrix
    kf.Q = np.eye(6) * 0.001

    # Measurement noise covariance matrix
    kf.R = np.eye(2) * 5
    kf.dt = dt

    return kf


if __name__ == "__main__":
    # Example of how to use the Kalman Filter

    # Create the Kalman Filter
    time_interval = 0.1  # Time step between measurements
    kf = create_kalman_filter(time_interval)

    # Initialize the state
    kf.x = np.array([0, 0, 0, 0, 0, 0])

    # Simulate receiving noisy position measurements
    noisy_measurements = [[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]]

    # Update the Kalman Filter with the measurements and predict the future position
    for position in noisy_measurements:
        # Predict the next state
        kf.predict()

        # Update the Kalman Filter with the new measurement
        kf.update(position)

        # Get the estimated state (position, velocity, acceleration)
        estimated_state = kf.x
        print("Estimated state:", estimated_state)

    # Predict the future position
    next_time_interval = 1  # Time step into the future
    position, velocity, acceleration = kf.x[:2], kf.x[2:4], kf.x[4:]
    predicted_x = (
        position[0] + velocity[0] * next_time_interval + 0.5 * acceleration[0] * next_time_interval**2
    )
    predicted_y = (
        position[1] + velocity[1] * next_time_interval + 0.5 * acceleration[1] * next_time_interval**2
    )
    predicted_position = np.array([predicted_x, predicted_y])
    print("Predicted future position:", predicted_position)
