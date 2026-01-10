package cereal

import (
	"log/slog"
	"math"

	"pfeifer.dev/mapd/cereal/log"
	m "pfeifer.dev/mapd/math"
)

type GpsSub struct {
	liveLocationKalman  Subscriber[log.LiveLocationKalman]
	gpsLocation         Subscriber[log.GpsLocationData]
	gpsLocationExternal Subscriber[log.GpsLocationData]
	useLLK              bool
	useExt              bool
}

// Read returns raw GPS data (for backwards compatibility)
func (s *GpsSub) Read() (locationData log.GpsLocationData, success bool) {
	if s.useExt {
		return s.gpsLocationExternal.Read()
	} else {
		locationData, success = s.gpsLocationExternal.Read()
		if success {
			s.useExt = true
			slog.Info("Found gpsLocationExternal, switching to external GPS provider")
			return locationData, success
		}
	}
	return s.gpsLocation.Read()
}

// ReadPosition returns position from the best available source
// Priority: LLK > gpsLocationExternal > gpsLocation
func (s *GpsSub) ReadPosition() (pos m.Position, bearingDeg float32, success bool) {
	llk, llkSuccess := s.liveLocationKalman.Read()
	if llkSuccess && llk.Status() == log.LiveLocationKalman_Status_valid {
		if !s.useLLK {
			s.useLLK = true
			slog.Info("Found liveLocationKalman, using Kalman-filtered GPS")
		}
		pos, ok := m.PosFromLLK(llk)
		if ok {
			bearing := llkBearing(llk)
			return pos, bearing, true
		}
	}

	// Fallback to gpsLocationExternal
	if s.useExt {
		loc, ok := s.gpsLocationExternal.Read()
		if ok {
			return m.PosFromLocation(loc), loc.BearingDeg(), true
		}
	} else {
		loc, ok := s.gpsLocationExternal.Read()
		if ok {
			s.useExt = true
			slog.Info("Found gpsLocationExternal, switching to external GPS provider")
			return m.PosFromLocation(loc), loc.BearingDeg(), true
		}
	}

	loc, ok := s.gpsLocation.Read()
	if ok {
		return m.PosFromLocation(loc), loc.BearingDeg(), true
	}

	return m.Position{}, 0, false
}

func llkBearing(llk log.LiveLocationKalman) float32 {
	velNED, err := llk.VelocityNED()
	if err != nil || !velNED.Valid() {
		return 0
	}
	values, err := velNED.Value()
	if err != nil || values.Len() < 2 {
		return 0
	}
	vN := values.At(0) // North velocity
	vE := values.At(1) // East velocity

	// Bearing = atan2(vE, vN) in degrees
	bearing := float32(math.Atan2(vE, vN) * 180.0 / math.Pi)
	if bearing < 0 {
		bearing += 360
	}
	return bearing
}

func (s *GpsSub) IsUsingLLK() bool {
	return s.useLLK
}

func (s *GpsSub) Close() {
	s.liveLocationKalman.Sub.Msgq.Close()
	s.gpsLocation.Sub.Msgq.Close()
	s.gpsLocationExternal.Sub.Msgq.Close()
}

func GetGpsSub() (gpsSub GpsSub) {
	return GpsSub{
		liveLocationKalman:  NewSubscriber("liveLocationKalman", LiveLocationKalmanReader, true),
		gpsLocation:         NewSubscriber("gpsLocation", GpsLocationReader, true),
		gpsLocationExternal: NewSubscriber("gpsLocationExternal", GpsLocationExternalReader, true),
		useLLK:              false,
		useExt:              false,
	}
}
