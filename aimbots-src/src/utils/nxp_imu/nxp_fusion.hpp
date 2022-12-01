#pragma once

//
// We totally yoinked all this code from an absolute legend on GitHub. The
// code is here: https://github.com/PaulStoffregen/NXPMotionSense
//

#include <cstdint>

namespace utils {

class NXPSensorFusion {
public:
	void begin(float sampleRate = 1000.0f);
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	// TODO: these aren't the same ranges as Madgwick & Mahony... or are they??
	float getRoll() { return PhiPl; }
	float getPitch() { return ThePl; }
	float getYaw() { return PsiPl; }
        float getOmegaX() { return Omega[0]; }
        float getOmegaY() { return Omega[1]; }
        float getOmegaZ() { return Omega[2]; }
	typedef struct {
		float q0; // w
		float q1; // x
		float q2; // y
		float q3; // z
	} Quaternion_t;
	void getQuaternion(float quat[4]) {
		quat[0] = qPl.q0;
		quat[1] = qPl.q1;
		quat[2] = qPl.q2;
		quat[3] = qPl.q3;
	}
	// These are Madgwick & Mahony - extrinsic rotation reference (wrong!)
	//float getPitch() {return atan2f(2.0f * qPl.q2 * qPl.q3 - 2.0f * qPl.q0 * qPl.q1, 2.0f * qPl.q0 * qPl.q0 + 2.0f * qPl.q3 * qPl.q3 - 1.0f);};
	//float getRoll() {return -1.0f * asinf(2.0f * qPl.q1 * qPl.q3 + 2.0f * qPl.q0 * qPl.q2);};
	//float getYaw() {return atan2f(2.0f * qPl.q1 * qPl.q2 - 2.0f * qPl.q0 * qPl.q3, 2.0f * qPl.q0 * qPl.q0 + 2.0f * qPl.q1 * qPl.q1 - 1.0f);};

private:
        float PhiPl;                    // roll (deg)
        float ThePl;                    // pitch (deg)
        float PsiPl;                    // yaw (deg)
        float RhoPl;                    // compass (deg)
        float ChiPl;                    // tilt from vertical (deg)
        // orientation matrix, quaternion and rotation vector
        float RPl[3][3];                // a posteriori orientation matrix
        Quaternion_t qPl;               // a posteriori orientation quaternion
        float RVecPl[3];                // rotation vector
        // angular velocity
        float Omega[3];                 // angular velocity (deg/s)
        // systick timer for benchmarking
        int32_t systick;                // systick timer;
        // end: elements common to all motion state vectors

        // elements transmitted over bluetooth in kalman packet
        float bPl[3];                   // gyro offset (deg/s)
        float ThErrPl[3];               // orientation error (deg)
        float bErrPl[3];                // gyro offset error (deg/s)
        // end elements transmitted in kalman packet

        float dErrGlPl[3];              // magnetic disturbance error (uT, global frame)
        float dErrSePl[3];              // magnetic disturbance error (uT, sensor frame)
        float aErrSePl[3];              // linear acceleration error (g, sensor frame)
        float aSeMi[3];                 // linear acceleration (g, sensor frame)
        float DeltaPl;                  // inclination angle (deg)
        float aSePl[3];                 // linear acceleration (g, sensor frame)
        float aGlPl[3];                 // linear acceleration (g, global frame)
        float gErrSeMi[3];              // difference (g, sensor frame) of gravity vector (accel) and gravity vector (gyro)
        float mErrSeMi[3];              // difference (uT, sensor frame) of geomagnetic vector (magnetometer) and geomagnetic vector (gyro)
        float gSeGyMi[3];               // gravity vector (g, sensor frame) measurement from gyro
        float mSeGyMi[3];               // geomagnetic vector (uT, sensor frame) measurement from gyro
        float mGl[3];                   // geomagnetic vector (uT, global frame)
        float QvAA;                     // accelerometer terms of Qv
        float QvMM;                     // magnetometer terms of Qv
        float PPlus12x12[12][12];       // covariance matrix P+
        float K12x6[12][6];             // kalman filter gain matrix K
        float Qw12x12[12][12];          // covariance matrix Qw
        float C6x12[6][12];             // measurement matrix C
        float RMi[3][3];                // a priori orientation matrix
        Quaternion_t Deltaq;            // delta quaternion
        Quaternion_t qMi;               // a priori orientation quaternion
        float casq;                     // FCA * FCA;
        float cdsq;                     // FCD * FCD;
        float Fastdeltat;               // sensor sampling interval (s) = 1 / SENSORFS
        float deltat;                   // kalman filter sampling interval (s) = OVERSAMPLE_RATIO / SENSORFS
        float deltatsq;                 // fdeltat * fdeltat
        float QwbplusQvG;               // FQWB + FQVG
        int8_t FirstOrientationLock;    // denotes that 9DOF orientation has locked to 6DOF
        int8_t resetflag;               // flag to request re-initialization on next pass
};

}