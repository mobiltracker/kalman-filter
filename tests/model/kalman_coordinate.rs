use std::ops::{Deref, DerefMut};

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KalmanCoordinate {
    pub received_time: DateTime<Utc>,
    pub server_time: DateTime<Utc>,
    pub collected_time: DateTime<Utc>,
    pub tracker_id: i64,
    pub accuracy: f64,
    pub lat: f64,
    pub lon: f64,
    pub speed: f64,
    pub event: String,
    pub acc_status: Option<bool>,
    pub battery: Option<i32>,
    pub imei: Option<String>,
    pub alarm_type: Option<i32>,
    pub kalman_info: KalmanInfo,
}
#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KalmanInfo {
    latitude: f64,
    longitude: f64,
    speed: f32,
    epsilon: f32,
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KalmanCoordinates(pub Vec<KalmanCoordinate>);

impl From<Vec<KalmanCoordinate>> for KalmanCoordinates {
    fn from(data: Vec<KalmanCoordinate>) -> Self {
        KalmanCoordinates(data)
    }
}

impl IntoIterator for KalmanCoordinates {
    type Item = KalmanCoordinate;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

impl Deref for KalmanCoordinates {
    type Target = Vec<KalmanCoordinate>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for KalmanCoordinates {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
