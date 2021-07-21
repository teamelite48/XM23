
/*
	Credit due to:
 	TEAM HORNET: 2603
 	(highlandrobotics.com)

	Traction control:
 	Implemented by Nicholas Tietz
 	(ntietz@gmail.com)
	
	Known bugs:
 	Distance per tick is wrong; it should use 0.1524*pi*(15/22), I forgot to put the pi in. Oddly enough, it works wonderfully.

	Questions/comments:
 	Please forward to ntietz@gmail.com
 	I would be glad to hear about it if my code can help anyone. (Or if you find some errors.)

	Anyone is welcome to use this code, but please give due credit.

 */

#include "AugmentedEncoder.h"
#include "frc/Encoder.h"
#include "frc/Timer.h"


	AugmentedEncoder::AugmentedEncoder(int a_channel, int b_channel, float d_p_t, bool reverse = false) 
	{
	//initializer for the AugmentedEncoder class
		encoder = new frc::Encoder(a_channel, b_channel, reverse);
		timer = new frc::Timer();
		mAugEncoderState = k1X;
		delta_d = 0;
		delta_v = 0;
		delta_t = 0;
		distance = 0;
		velocity = 0;
		acceleration = 0;
		distance_per_tick = d_p_t;
	} //end AugmentedEncoder(...)
	
	void AugmentedEncoder::Start() {
	//starts the encoder and timer
		//encoder->Start();
		timer->Start();
	}
	void AugmentedEncoder::Recalculate() {
	//calculates changes of distance, velocity, and time, as well as absolute velocity and acceleration.
		delta_t = timer->Get(); //time elapsed since last recalculation
			timer->Reset(); //resets the time elapsed
		//delta_d = encoder->Get() * distance_per_tick / 4; //quadrature gives 4 times resolution but requires division by 4
		delta_d = encoder->Get() * distance_per_tick; 	
		delta_v = delta_d / delta_t - velocity; //delta_d / delta_t is current velocity
		distance = encoder->Get() * distance_per_tick;
		velocity += delta_v; //current velocity is now set to old velocity plus the change
		acceleration = delta_v / delta_t; //acceleration is rate of change of velocity
		encoder->Reset(); //resets the ticks for the encoder
	
	}
	void AugmentedEncoder::Reset() {
	//resets the augmented encoder
		velocity = acceleration = distance = 0.0;
		timer->Reset();
		encoder->Reset();		
	}
	float AugmentedEncoder::GetAcceleration()
	{
		return acceleration; //returns a private member
	}
	float AugmentedEncoder::GetVelocity()
	{
		return velocity; //returns a private member
	}
	float AugmentedEncoder::GetDistance()
	{
		distance = encoder->Get() * distance_per_tick;
		return distance; //returns a private number
	}
	










