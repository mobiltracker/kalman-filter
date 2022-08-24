mod model;
use std::fs;

use kalman_filter::{
    KalmanFilter, ACCELERATION_STD_DEFAULT, INITIAL_UNCERTAINTY_DEFAULT, POSITION_STD_DEFAULT,
};
use model::kalman_coordinate::KalmanCoordinates;

fn kalman_filter(coordinates: KalmanCoordinates) {
    let initial_uncertainty = INITIAL_UNCERTAINTY_DEFAULT;
    let acc_std = ACCELERATION_STD_DEFAULT;
    let position_std = POSITION_STD_DEFAULT;

    let mut last_time = coordinates[0].received_time;
    let mut kalman_filter = KalmanFilter::builder(
        geo::Coordinate {
            x: coordinates[0].lon as f32,
            y: coordinates[0].lat as f32,
        },
        initial_uncertainty,
    )
    .uncertainty_increase_multiplier(10000f32)
    .build();

    for coordinate in coordinates[1..].into_iter() {
        let elapsed_time = coordinate.received_time - last_time;
        last_time = coordinate.received_time;
        kalman_filter.predict(elapsed_time.num_seconds() as f32, acc_std);

        let coordinate = geo::Coordinate {
            x: coordinate.lon as f32,
            y: coordinate.lat as f32,
        };
        kalman_filter.update(coordinate, position_std);
    }
}

#[test]
fn read_and_test_file() {
    let raw = fs::read_to_string("tests/data/lh-kalman-another-example.json")
        .expect("Couldn't read file");

    let coordinates: KalmanCoordinates = serde_json::from_str(&raw).expect("Couldn't parse json");
    kalman_filter(coordinates);
}
