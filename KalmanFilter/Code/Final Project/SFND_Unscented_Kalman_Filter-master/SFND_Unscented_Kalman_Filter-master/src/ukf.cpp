#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
// Student Code Start

// Calculate NIS values
inline const float UKF::CalculateNISValue(const VectorXd z_prediction, const VectorXd z_measurement,
                                          const MatrixXd covariance) {
    VectorXd difference{ z_measurement - z_prediction };
    return difference.transpose() * covariance.inverse() * difference;
}


// Function to check if we Lidar measurement is going to be used or not
inline const bool UKF::isUsingLaser(const MeasurementPackage &meas_package) {
    return meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_;
}

// Function to check if we Radar measurement is going to be used or not
inline const bool UKF::isUsingRadar(const MeasurementPackage &meas_package) {
    return meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_;
}

// Function to keep yaw angle between 0 and -pi
inline void UKF::decrementNormalize(VectorXd &vec, const int index) {
    while (vec(index) > M_PI) 
        vec(index) -= twice_m_pi; 
}

// Function to keep yaw angle between 0 and pi
inline void UKF::incrementNormalize(VectorXd &vec, const int index, const bool negativePi = false) {
    if (negativePi) 
    {
        while (vec(index) < -M_PI) 
            vec(index) += twice_m_pi; 
    } 
    else 
    {
        while (vec(index) < M_PI) 
            vec(index) += twice_m_pi; 
    }
}

// Student Code End

UKF::UKF() {
    n_x_ = 5;
    n_aug_ = 7;
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(n_x_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;
  
    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
  
    /**
     * End DO NOT MODIFY section for measurement noise values 
     */
    
    /**
     * TODO: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */
    aug_dim = 2 * n_aug_ + 1;
    std_a_ = 3.0;     // Process noise standard deviation longitudinal acceleration in m/s^2
    std_yawdd_ = 1.0; // Process noise standard deviation yaw acceleration in rad/s^2
    Xsig_pred_ = MatrixXd(n_x_, aug_dim); // predicted sigma points matrix
    weights_ = VectorXd(aug_dim);         // create vector for weights
    lambda_ = 3 - n_aug_;                 // Sigma point spreading parameter
    is_initialized_ = false;
}

UKF::~UKF() {
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
    /**
     * Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */
    

    constexpr int RADIAL_DIST = 0;
    constexpr int BEARING_VEL = 1;
    constexpr int RADIAL_VEL = 2;

    if (!is_initialized_) 
    {
        // Getting measurements radial distance and bearing velocity from meas_package
        const double radial_diff = meas_package.raw_measurements_[RADIAL_DIST];
        const double bearing_angle = meas_package.raw_measurements_[BEARING_VEL];
        // Checking if we have to work with Radar or Lidar.
        if (isUsingRadar(meas_package)) 
        {
            // Getting measurement of radial velocity from meas_package 
            const double radial_velocity = meas_package.raw_measurements_[RADIAL_VEL];
            // const double sin_radial_diff = sin(radial_diff);
            const double sin_bearing_angle = sin(bearing_angle);
            const double cos_bearing_angle = cos(bearing_angle);
            
            // Calculating velocity of the sensed vehicle. 
            const double velocity = sqrt(
                radial_velocity * sin_bearing_angle * radial_velocity * sin_bearing_angle +
                radial_velocity * cos_bearing_angle * radial_velocity * cos_bearing_angle);
            
            // Initialising state covariance matrix to be Identity. It is considered 5*5 as we have 5 state values, namely, position in x and y, velocity, yaw angle and yaw rate.
            P_ << 1, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 
                  0, 0, 1, 0, 0, 
                  0, 0, 0, 1, 0, 
                  0, 0, 0, 0, 1;
            x_ << (radial_diff * cos_bearing_angle), 
                  (radial_diff * sin_bearing_angle), 
                                           velocity, 
                                                  0, 
                                                  0;
        } 
        else if (isUsingLaser(meas_package)) // Checking if we have to work with Radar or Lidar
        {
            // Lidar will be giving distance to the object
            x_ << radial_diff, 
                bearing_angle, 
                            0, 
                            0, 
                            0;

            const double std_laspx_squared = std_laspx_ * std_laspx_;
            // Initialising state covariance matrix. Position in x and y direction have same sigma square.           
            P_ << std_laspx_squared, 0, 0, 0, 0, 
                  0, std_laspx_squared, 0, 0, 0, 
                                  0, 0, 5, 0, 0, 
                                  0, 0, 0, 1, 0, 
                                  0, 0, 0, 0, 1;
        }
        // Getting the time stamp of measurement update.
        prior_time_stamp = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }
    // n_aug_ is the number of augmented state, we need to consider for process update. 
    const double divisor = lambda_ + n_aug_ ; 

    weights_.fill(0.0);
    weights_(0) = (lambda_ / divisor);
    
    for (int index = 1; index < aug_dim; index++) 
        weights_(index) = 0.5 / divisor;
    
    // Calculating time delta between previous and current measurement update.
    const double time_delta = (meas_package.timestamp_ - prior_time_stamp) / 1000000.0;
    prior_time_stamp = meas_package.timestamp_;
    // Using the time delta to do the Prediction. 
    Prediction(time_delta);
    
    // Checking if we have used Lidar or Radar update in previous steps. Depending on the use case, we will update system state.
    if (isUsingLaser(meas_package)) 
    {
        UpdateLidar(meas_package);

    } else if (isUsingRadar(meas_package)) 
    {
        UpdateRadar(meas_package);
    }
}

void UKF::Prediction(double delta_t) {
    /**
     * Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
    // Initialising the Augmented state, state covariance matriix and sigma points for augmented state (2*7+1).
    VectorXd x_augmented_matrix = VectorXd(n_aug_); // 7 states
    MatrixXd P_augmented_matrix = MatrixXd(n_aug_, n_aug_); // 7*7 augmented matrix
    MatrixXd x_sigma_augmented = MatrixXd(n_aug_, aug_dim); // 7*(2*7+1) = 7*15 dimension matrix to store sigma points. Each sigma point is in a column. aug_dum = 15

    x_augmented_matrix.head(n_x_) = x_; // Top 5 states are same as before augmentation
    x_augmented_matrix(n_x_) = 0; // 6th state is zero 
    x_augmented_matrix(n_x_ + 1) = 0; // 7th state is also zero
    
    // Initialising a Augmented State Covariance Matrix P_augmented_matrix and filling the top left 5*5 matrix with P_ and the 6 and 7 the diagonal element filled with standard deviation square of linear and yaw acceleration.
    P_augmented_matrix.fill(0);
    P_augmented_matrix.topLeftCorner(n_x_, n_x_) = P_;
    P_augmented_matrix(n_x_, n_x_) = std_a_ * std_a_;
    P_augmented_matrix(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    x_sigma_augmented.fill(0);
    x_sigma_augmented.col(0) = x_augmented_matrix; // First column has current augmented state vector as sigma point, rest sigma points are calculated below.
    
    // n_aug_ seems to be 7, as we use (lambda+7) to find sigma points.
    const double shift = sqrt(lambda_ + n_aug_); 
    
    // Implmentation to calculate sigma points. 
    MatrixXd sqrt_matrix = P_augmented_matrix.llt().matrixL();
    // Looping 7 times, so that 14 sigma points are found (total sigma points are 15).
    for (int index = 0; index < n_aug_; index++) // n_aug_ = 7
    {
        x_sigma_augmented.col(index + 1) = x_augmented_matrix + shift * sqrt_matrix.col(index);
        x_sigma_augmented.col(index + 1 + n_aug_) = x_augmented_matrix - shift * sqrt_matrix.col(index);
    }
    
    // Loop over all the sigma points. All the sigma points need to be predicted
    for (int index{ 0 }; index < aug_dim; index++) 
    {
        const double x_point = x_sigma_augmented(0, index);
        const double y_point = x_sigma_augmented(1, index);
        const double velocity = x_sigma_augmented(2, index);
        const double yaw = x_sigma_augmented(3, index);
        const double yaw_diff = x_sigma_augmented(4, index);
        const double nu_angle = x_sigma_augmented(5, index);
        const double nu_yaw_diff = x_sigma_augmented(6, index);

        double predicted_x, predicted_y;
        // Checking if yaw rate is small enough or not. Different equation will have to be considered if yaw rate is very small.
        // First two predicted states.
        if (fabs(yaw_diff) > 0.001) 
        {
            predicted_x = x_point + velocity / yaw_diff * (sin(yaw + yaw_diff * delta_t) - sin(yaw));
            predicted_y = y_point + velocity / yaw_diff * (cos(yaw) - cos(yaw + yaw_diff * delta_t));
        } 
        else 
        {
            predicted_x = x_point + velocity * delta_t * cos(yaw);
            predicted_y = y_point + velocity * delta_t * sin(yaw);
        }
        // Other predicted states. (Prediction only by dynamics)
        double predicted_velocity = velocity;
        double yaw_predicted = yaw + yaw_diff * delta_t;
        double yaw_diff_predicted = yaw_diff;

        // Error terms added here. These terms are due to acceleration
        const double predicate = 0.5 * nu_angle * delta_t * delta_t;
        predicted_x += predicate * cos(yaw);
        predicted_y += predicate * sin(yaw);
        predicted_velocity += nu_angle * delta_t;
        yaw_predicted += 0.5 * nu_yaw_diff * delta_t * delta_t;
        yaw_diff_predicted += nu_yaw_diff * delta_t;
        
        // The first 5 predicted states are stored in Xsig_pred_.
        Xsig_pred_(0, index) = predicted_x;
        Xsig_pred_(1, index) = predicted_y;
        Xsig_pred_(2, index) = predicted_velocity;
        Xsig_pred_(3, index) = yaw_predicted;
        Xsig_pred_(4, index) = yaw_diff_predicted;
    }

    x_.fill(0.0);
    
    // Looping over all the sigma points to find weighted average
    for (int index{ 0 }; index < aug_dim; index++) 
        x_ += weights_(index) * Xsig_pred_.col(index); 

    P_.fill(0.0);
    // Looping over all the sigma points to normalise the yaw angle. It should be between -pi to pi
    for (int index{ 0 }; index < aug_dim; index++) 
    {
        VectorXd x_angle_difference = Xsig_pred_.col(index) - x_;
        decrementNormalize(x_angle_difference, 3);
        incrementNormalize(x_angle_difference, 3, true);
        // Calculating the new state covariance matrix
        P_ += weights_(index) * x_angle_difference * x_angle_difference.transpose();
    }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     * Lidar data is used to update the belief about the object's position.
     * Modifications: the state vector, x_, and covariance, P_.
     * NIS, if desired.
     */

    // Initialising H matrix.
    MatrixXd H = MatrixXd(2, n_x_); // n_x_ is 5
    // Looks like H matrix.
    H << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0;

    // Lidar measurement covariance matrix 2*2
    MatrixXd lidar_covariance = MatrixXd(2, 2);
    lidar_covariance << std_laspx_ * std_laspx_, 0, 
                        0, std_laspy_ * std_laspy_;
    // Reading the Lidar measurement in z_vector
    VectorXd z_vector = meas_package.raw_measurements_ ;
    VectorXd z_prediction = H * x_;
    VectorXd y_vector = z_vector - z_prediction;
    MatrixXd H_transpose = H.transpose();
    MatrixXd measurement_covariance = H * P_ * H_transpose + lidar_covariance;
    MatrixXd measurement_covariance_inverse = measurement_covariance.inverse();
    MatrixXd gain = P_ * H_transpose * measurement_covariance_inverse;
    
    // State update after considering measurements
    x_ += gain * y_vector;
    
    // State covariance matrix update after considering lidar measurement covariance matrix
    MatrixXd identity_matrix = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (identity_matrix - gain * H) * P_;
    
    // Calculating NIS value
    std::cout << "Lidar NIS: " << CalculateNISValue(z_prediction, z_vector, measurement_covariance)
              << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     * Radar data is used to update the belief about the object's position.
     * Modified: the state vector, x_, and covariance, P_.
     * NIS, if desired.
     */
    constexpr int measurement_dim = 3;
    // Initialisaing variables for storing the sigma_points vector for measurement model projection.
    MatrixXd z_sigma = MatrixXd(measurement_dim, aug_dim); // 3*15 dimension. That is total 15 sigma points of state dimension 3
    VectorXd z_prediction = VectorXd(measurement_dim);
    MatrixXd measurement_covariance = MatrixXd(measurement_dim, measurement_dim); // 3*3
    
    // Looping over all the sigma points. For each sigma point that we got from process model projection, we are finding radial distance, yaw angle and yaw rate.
    for (int index{ 0 }; index < aug_dim; index++) 
    {
        const double x_point = Xsig_pred_(0, index);
        const double y_point = Xsig_pred_(1, index);
        const double velocity = Xsig_pred_(2, index);
        const double yaw = Xsig_pred_(3, index);
        
        // Velocity in x and y direction.
        const double velocity_x = velocity * cos(yaw);
        const double velocity_y = velocity * sin(yaw);
        // radial distance to object
        const double dist = sqrt(x_point * x_point + y_point * y_point);

        z_sigma(0, index) = dist;
        z_sigma(1, index) = atan2(y_point, x_point);
        z_sigma(2, index) = (x_point * velocity_x + y_point * velocity_y) / dist;
    }

    z_prediction.fill(0);
    
    // Finding the weighted average of the sigma points
    for (int index{ 0 }; index < aug_dim; index++) 
        z_prediction += weights_(index) * z_sigma.col(index); 

    measurement_covariance.fill(0);
    // Again looping over all the sigma points and making sure that each sigma point after measurement model projection has yaw angle between -pi and pi. 
    for (int index{ 0 }; index < aug_dim; index++) 
    {
        VectorXd z_angle_difference = z_sigma.col(index) - z_prediction;
        decrementNormalize(z_angle_difference, 1);
        incrementNormalize(z_angle_difference, 1);
        // Finally finding predicted measurement covariance using the sigma points generated.
        measurement_covariance += weights_(index) * z_angle_difference * z_angle_difference.transpose();
    }

    // Measurement noise covariance matrix
    MatrixXd noise = MatrixXd(measurement_dim, measurement_dim);
    const double squared_std_radr = std_radr_ * std_radr_;
    noise << squared_std_radr, 0, 0, 
             0, squared_std_radr, 0, 
             0, 0, squared_std_radr;
    
    // Adding measurement noise covariance to the updated predicted measurement covariance
    measurement_covariance = measurement_covariance + noise;

    // Cross Correlation matrix T
    MatrixXd cross_correlation = MatrixXd(n_x_, measurement_dim);
    cross_correlation.fill(0);
    // Looping over all the sigma points to find Cross Correlation matrix T. It is needed to get the updated state covariance matrix
    for (int index{ 0 }; index < aug_dim; index++) 
    {
        VectorXd z_angle_difference = z_sigma.col(index) - z_prediction;
        // Normalising for, sigma points projection through measurement model
        decrementNormalize(z_angle_difference, 1);
        incrementNormalize(z_angle_difference, 1);

        VectorXd x_angle_difference = Xsig_pred_.col(index) - x_;
        // Normalising for, sigma points projection through process model
        decrementNormalize(x_angle_difference, 3);
        incrementNormalize(x_angle_difference, 3, true);
        // Calculation of Cross Correlation matrix
        cross_correlation += weights_(index) * x_angle_difference * z_angle_difference.transpose();
    }

    MatrixXd gain = cross_correlation * measurement_covariance.inverse(); // It is infact Kalman Gain matrix

    VectorXd z_vector = meas_package.raw_measurements_ ;
    VectorXd z_angle_difference = z_vector - z_prediction;
    // Again normalising the Yaw angle
    incrementNormalize(z_angle_difference, 1);
    decrementNormalize(z_angle_difference, 1);
    // Finally updating state and state covariance matrix.
    x_ = x_ + gain * z_angle_difference;
    P_ = P_ - gain * measurement_covariance * gain.transpose();

    std::cout << "Radar NIS: " << CalculateNISValue(z_prediction, z_vector, measurement_covariance) << std::endl;
}


