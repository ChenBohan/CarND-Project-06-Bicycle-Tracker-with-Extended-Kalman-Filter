# CarND-Project-06-Bicycle-Tracker-with-Extended-Kalman-Filter
Udacity Self-Driving Car Engineer Nanodegree: Project 5 - Extended Kalman Filter

## Further reading

- [EKF project Github repository README](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

- [Download the simulator](https://github.com/udacity/self-driving-car-sim/releases/)

## Files in the Github src Folder

- `main.cpp` - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE

- `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function

- `kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar

- `tools.cpp` - function to calculate RMSE and the Jacobian matrix

## Data File

data file: `obj_pose-laser-radar-synthetic-input.txt`

Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

## Kalman Filter

### 1. Initialize

1. Initialize the state ekf_.x_ with the first measurement.

2. Create the covariance matrix.

```pyhon
ekf_.P_ = MatrixXd(4, 4);
ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;
```

3. Convert radar from polar to cartesian coordinates.

```python
if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  float rho     = measurement_pack.raw_measurements_(0);
  float phi     = measurement_pack.raw_measurements_(1);
  float rho_dot = measurement_pack.raw_measurements_(2);
  ekf_.x_(0) = rho     * cos(phi);
  ekf_.x_(1) = rho     * sin(phi);      
  ekf_.x_(2) = rho_dot * cos(phi);
  ekf_.x_(3) = rho_dot * sin(phi);
}
else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
  ekf_.x_(0) = measurement_pack.raw_measurements_(0);
  ekf_.x_(1) = measurement_pack.raw_measurements_(1);
  ekf_.x_(2) = 0.0;
  ekf_.x_(3) = 0.0;
}
```

### 2. Predict

### 3. Update
