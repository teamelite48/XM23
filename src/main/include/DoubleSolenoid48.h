#ifndef DOUBLE_SOLENOID_H_
#define DOUBLE_SOLENOID_H_

#include "frc/Solenoid.h"

/**
 * Wraps two Solenoid objects into a single class for double solenoids.
 */
class DoubleSolenoid48
{
public:
	typedef enum { OnA, OnB, kOff } DoubleSolenoid48State;
	DoubleSolenoid48(uint32_t aChannel, uint32_t bChannel);
	DoubleSolenoid48(uint8_t aModule, uint32_t aChannel, uint8_t bModule, uint32_t bChannel);
	~DoubleSolenoid48();
	void Set(DoubleSolenoid48State state);
	DoubleSolenoid48State Get();

private:
	frc::Solenoid *aSolenoid;
	frc::Solenoid *bSolenoid;
};

#endif // DOUBLE_SOLENOID_H_
