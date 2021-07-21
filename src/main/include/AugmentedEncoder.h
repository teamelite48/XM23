#ifndef XM19_AUGMENTEDENCODER_H_
#define XM19_AUGMENTEDENCODER_H_

#include "frc/Encoder.h"
#include "frc/Timer.h"


class AugmentedEncoder {
//augments the functionality of an encoder

	float acceleration;
	float velocity;
	float distance;
	float delta_v; //change in velocity
	float delta_d; //change in distance
	float delta_t; //change in time
	float distance_per_tick; //distance per tick of the encoder

public:
	
	typedef enum {
		k1X, k2X, k4X
	} AugEncoderState;
	
	AugmentedEncoder(int a_channel, int b_channel, float d_p_t, bool reverse);
	void Start();
	void Recalculate();
	void Reset();
	float GetAcceleration();
	float GetVelocity();
	float GetDistance();
	
private:

	frc::Encoder *encoder;
	frc::Timer *timer;
	AugEncoderState mAugEncoderState;
	
};

#endif // XM19_AUGMENTEDENCODER_H_
