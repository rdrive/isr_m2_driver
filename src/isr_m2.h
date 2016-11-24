#ifndef __ISR_M2_H__
#define __ISR_M2_H__

#define WHEEL_RADIUS_M 0.155	// Unit:m
#define WHEEL_BASE_M 0.531 // Unit:m
#define WHEEL_WIDTH_M 0.102 // Unit:m
#define ENCODER_PPR 6400.0	// Unit: pulse/rev
#define GEAR_RATIO 31.778	// Gearhead reduction ratio: 26 (26:1), Spurgear reduction ratio: 1.22 (44:36)
#define MPS2RPM 61.608 // same as (60 /(2 * M_PI * WHEEL_RADIUS_M))
#define MAX_RPM 4650.0

namespace rdrive {
namespace isr_m2 {

class ISR_M2
{
public:
	ISR_M2();
	~ISR_M2();

	bool Initialize(void);
	bool ResetRobotPos(void);
	void SetEncoderValue(long l_count, long r_count, int t_ms);

public:
	struct Position
	{
		double x;
		double y;
		double theta;
	} Position; // Robot pose calculated by dead-reckoning (Unit: m, m, rad)

private:
	void DeadReckoning(long dl, long dr, int dt);

private:
	int _readEncoderValueTime_ms;		// ms
	int _readEncoderValueTime_ms_prev;	// ms

	long _leftEncoderValue_prev;	// A previous encoder value of left wheel (Unit: pulse)
	long _rightEncoderValue_prev;	// A previous encoder value of right wheel (Unit: pulse)
	long _leftEncoderValue;			// A encoder value of left wheel (Unit: pulse)
	long _rightEncoderValue;		// A encoder value of right wheel (Unit: pulse)
};

} //namespace isr_m2
} // namespace rdrive

#endif // __ISR_M2_H__