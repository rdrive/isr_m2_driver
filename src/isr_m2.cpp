#include "isr_m2.h"
#include <math.h>

namespace rdrive {
namespace isr_m2 {

ISR_M2::ISR_M2()
{
}

ISR_M2::~ISR_M2()
{
}

bool ISR_M2::Initialize(void)
{
	_readEncoderValueTime_ms_prev = 0;
	_readEncoderValueTime_ms = 0;

	_leftEncoderValue_prev = 0;
	_rightEncoderValue_prev = 0;
	_leftEncoderValue = 0;
	_rightEncoderValue = 0;

	ResetRobotPos();

	return true;
}

bool ISR_M2::ResetRobotPos(void)
{
	Position.x = Position.y = Position.theta = 0.0;
	return true;
}

void ISR_M2::SetEncoderValue(long l_count, long r_count, int t_ms)
{
	long dl, dr;
	int dt;

	_leftEncoderValue_prev = _leftEncoderValue;
	_rightEncoderValue_prev = _rightEncoderValue;
	_readEncoderValueTime_ms_prev = _readEncoderValueTime_ms;
	_leftEncoderValue = l_count;
	_rightEncoderValue = r_count;
	_readEncoderValueTime_ms = t_ms;
	dl = _leftEncoderValue - _leftEncoderValue_prev;
	dr = _rightEncoderValue - _rightEncoderValue_prev;
	dt = _readEncoderValueTime_ms - _readEncoderValueTime_ms_prev;

	DeadReckoning(dl, dr, dt);
}

// Calculates forward kinematics to get robot's pose(x, y, theta) by changing amount of L/R wheel encoder.
void ISR_M2::DeadReckoning(long dl, long dr, int dt)
{
	if (dt <= 0) return;

	double r = 2.0 * M_PI*WHEEL_RADIUS_M / ENCODER_PPR / GEAR_RATIO;
	double delDistLeft_m = r*dl;
	double delDistRight_m = r*dr;
	double leftWheelVelocity_mps = delDistLeft_m / (double)dt * 1000.0; //m/s
    double rightWheelVelocity_mps = delDistRight_m / (double)dt * 1000.0;
	double v = (delDistLeft_m + delDistRight_m) / 2.0;
	double w = (delDistRight_m - delDistLeft_m) / WHEEL_BASE_M;

	if (-0.001 > w || w > 0.001)
	{
		Position.x += v / w*(sin(Position.theta + w) - sin(Position.theta));
		Position.y -= v / w*(cos(Position.theta + w) - cos(Position.theta));
	}
	else
	{
		Position.x += v*cos(Position.theta);
		Position.y += v*sin(Position.theta);
	}
	Position.theta += w;
}

} //namespace isr_m2
} // namespace rdrive
