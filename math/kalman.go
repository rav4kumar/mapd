package math

import (
	"math"

	ms "pfeifer.dev/mapd/settings"
)

const (
	// GPS accuracy ~3m = ~0.00003 degrees
	GPS_NOISE_DEG = 0.00003
	// Meters per degree latitude
	DEG_TO_M = 111000.0
)

// PositionKalman is a 4-state Kalman filter for GPS position smoothing
// State: [lat, lon, v_lat, v_lon]
// Uses VEgo for prediction between GPS updates
type PositionKalman struct {
	// State: [lat, lon, v_lat, v_lon]
	X [4]float64

	// Covariance matrix (diagonal approximation)
	P [4]float64

	// Process noise
	Q [4]float64

	// Measurement noise (GPS)
	R [2]float64

	initialized bool
}

// NewPositionKalman creates a new GPS position Kalman filter
func NewPositionKalman() *PositionKalman {
	kf := &PositionKalman{}

	// Initial covariance (high uncertainty)
	kf.P = [4]float64{
		GPS_NOISE_DEG * GPS_NOISE_DEG, // lat variance
		GPS_NOISE_DEG * GPS_NOISE_DEG, // lon variance
		1e-8,                           // v_lat variance
		1e-8,                           // v_lon variance
	}

	// Process noise (state drift per second)
	kf.Q = [4]float64{1e-10, 1e-10, 1e-9, 1e-9}

	// GPS measurement noise (~3m accuracy)
	kf.R = [2]float64{
		GPS_NOISE_DEG * GPS_NOISE_DEG,
		GPS_NOISE_DEG * GPS_NOISE_DEG,
	}

	return kf
}

// Initialize sets initial position from first GPS reading
func (kf *PositionKalman) Initialize(lat, lon float64) {
	kf.X[0] = lat
	kf.X[1] = lon
	kf.X[2] = 0
	kf.X[3] = 0
	kf.initialized = true
}

// IsInitialized returns whether the filter has been initialized with GPS
func (kf *PositionKalman) IsInitialized() bool {
	return kf.initialized
}

// SetVelocity updates velocity state from car odometry and bearing
func (kf *PositionKalman) SetVelocity(vEgo float32, bearingDeg float64) {
	if !kf.initialized {
		return
	}
	bearingRad := bearingDeg * ms.TO_RADIANS
	// Convert m/s to degrees/s
	vDegPerSec := float64(vEgo) / DEG_TO_M

	// v_lat: north component
	kf.X[2] = vDegPerSec * math.Cos(bearingRad)
	// v_lon: east component (with latitude correction for spherical earth)
	latRad := kf.X[0] * ms.TO_RADIANS
	kf.X[3] = vDegPerSec * math.Sin(bearingRad) / math.Cos(latRad)
}

// Predict advances state using constant velocity model
// Call this every loop (~50Hz) with dt = time since last call
func (kf *PositionKalman) Predict(dt float64) Position {
	if !kf.initialized {
		return Position{}
	}

	// State transition: x = x + v*dt
	kf.X[0] += kf.X[2] * dt
	kf.X[1] += kf.X[3] * dt

	// Covariance update: P = P + Q*dt
	kf.P[0] += kf.Q[0] * dt
	kf.P[1] += kf.Q[1] * dt
	kf.P[2] += kf.Q[2] * dt
	kf.P[3] += kf.Q[3] * dt

	return NewPosition(kf.X[0], kf.X[1])
}

// Update corrects state with GPS measurement
// Call this when new GPS data arrives (~1Hz)
func (kf *PositionKalman) Update(lat, lon float64) Position {
	if !kf.initialized {
		kf.Initialize(lat, lon)
		return NewPosition(lat, lon)
	}

	// Innovation: y = z - H*x
	y0 := lat - kf.X[0]
	y1 := lon - kf.X[1]

	// Innovation covariance: S = P + R
	s0 := kf.P[0] + kf.R[0]
	s1 := kf.P[1] + kf.R[1]

	// Kalman gain: K = P / S
	k0 := kf.P[0] / s0
	k1 := kf.P[1] / s1

	// State update: x = x + K*y
	kf.X[0] += k0 * y0
	kf.X[1] += k1 * y1

	// Covariance update: P = (1-K)*P
	kf.P[0] *= (1 - k0)
	kf.P[1] *= (1 - k1)

	return NewPosition(kf.X[0], kf.X[1])
}

// Position returns current filtered position estimate
func (kf *PositionKalman) Position() Position {
	return NewPosition(kf.X[0], kf.X[1])
}
