use nalgebra::{Matrix2, Matrix2x4, Matrix4, Matrix4x2, SMatrix, SVector};

const DIMENSIONS: usize = 4;
const UNCERTAINTY_INCREASE_DEFAULT: f32 = 1f32;
const EPSILON_THRESHOLD_DEFAULT: f32 = 0.0000009f32;
pub const ACCELERATION_STD_DEFAULT: f32 = 0.000001f32;
pub const POSITION_STD_DEFAULT: f32 = 0.0001f32;
pub const INITIAL_UNCERTAINTY_DEFAULT: f32 = 0.0001f32;
pub const TIME_STEP_DEFAULT: f32 = 1f32;

pub struct KalmanFilterBuilder {
    uncertainty_increase_multiplier: Option<f32>,
    epsilon_threshold: Option<f32>,
    coordinate: geo::Coordinate<f32>,
    initial_uncertainty: f32,
}

impl KalmanFilterBuilder {
    pub fn uncertainty_increase_multiplier(self, value: f32) -> Self {
        Self {
            uncertainty_increase_multiplier: Some(value),
            ..self
        }
    }

    pub fn epsilon_threshold(self, value: f32) -> Self {
        Self {
            epsilon_threshold: Some(value),
            ..self
        }
    }

    pub fn build(self) -> KalmanFilter {
        let state = SVector::from([self.coordinate.x, self.coordinate.y, 0f32, 0f32]);
        let covariance = Matrix4::from_diagonal_element(self.initial_uncertainty);

        KalmanFilter {
            state,
            covariance,
            uncertainty_increase_mult: self
                .uncertainty_increase_multiplier
                .unwrap_or(UNCERTAINTY_INCREASE_DEFAULT),
            epsilon_threshold: self.epsilon_threshold.unwrap_or(EPSILON_THRESHOLD_DEFAULT),
            count: 0,
        }
    }
}

pub struct KalmanFilter {
    state: SVector<f32, DIMENSIONS>,
    covariance: SMatrix<f32, DIMENSIONS, DIMENSIONS>,
    uncertainty_increase_mult: f32,
    epsilon_threshold: f32,
    count: u32,
}

#[derive(Debug)]
pub struct KalmanState {
    pub longitude: f32,
    pub latitude: f32,
    pub lon_speed: f32,
    pub lat_speed: f32,
}

impl KalmanFilter {
    pub fn builder(
        coordinate: geo::Coordinate<f32>,
        initial_uncertainty: f32,
    ) -> KalmanFilterBuilder {
        KalmanFilterBuilder {
            coordinate,
            initial_uncertainty,
            epsilon_threshold: None,
            uncertainty_increase_multiplier: None,
        }
    }

    pub fn update(&mut self, measurement: geo::Coordinate<f32>, position_std: f32) {
        let measurement = SVector::from([measurement.x, measurement.y]);

        let observation_model = Matrix2x4::from_diagonal_element(1f32);

        let residual = measurement - observation_model * self.state;

        let observation_noise_cov = Matrix2::from([
            [position_std.powf(2f32), 0f32],
            [0f32, position_std.powf(2f32)],
        ]);

        let inovation_covariance =
            observation_model * self.covariance * observation_model.transpose()
                + observation_noise_cov;

        let inovation_covariance = inovation_covariance.try_inverse().unwrap();

        let kalman_gain = self.covariance * observation_model.transpose() * inovation_covariance;

        self.state += kalman_gain * residual;
        let identity = Matrix4::<f32>::identity();
        self.covariance = (identity - kalman_gain * observation_model) * self.covariance;

        let epsilon = residual.transpose() * inovation_covariance * residual;

        if epsilon.x > self.epsilon_threshold {
            self.covariance *= self.uncertainty_increase_mult;
            self.count += 1;
        } else if self.count > 0 {
            self.covariance /= self.uncertainty_increase_mult;
            self.count -= 1;
        }
    }

    pub fn predict(&mut self, time_var: f32, acc_std: f32) {
        let trans_model = Matrix4::new(
            1f32, 0f32, time_var, 0f32, //
            0f32, 1f32, 0f32, time_var, //
            0f32, 0f32, 1f32, 0f32, //
            0f32, 0f32, 0f32, 1f32, //
        );

        let noise_trans_model = Matrix4x2::new(
            time_var.powf(2f32) / 2f32,
            0f32,
            0f32,
            time_var.powf(2f32) / 2f32,
            time_var,
            0f32,
            0f32,
            time_var,
        );

        let noise_covariance = noise_trans_model * noise_trans_model.transpose() * acc_std;

        self.state = trans_model * self.state;

        self.covariance =
            (trans_model * self.covariance * trans_model.transpose()) + noise_covariance;
    }

    pub fn state(&self) -> KalmanState {
        KalmanState {
            longitude: self.state[0],
            latitude: self.state[1],
            lon_speed: self.state[2],
            lat_speed: self.state[3],
        }
    }
}
