#include "DoubleSolenoid48.h"

/**
 * Constructs an instance of the DoubleSolenoid48 class.
 * Passes the channels (relay output numbers) to the two member solenoid object constructors.
 * 
 * @param aChannel The channel of the first side of the double solenoid
 * @param bChannel The channel of the second side of the double solenoid
 */
DoubleSolenoid48::DoubleSolenoid48(uint32_t aChannel, uint32_t bChannel)
{
	aSolenoid = new frc::Solenoid(aChannel);
	bSolenoid = new frc::Solenoid(bChannel);
}

/**
 * Constructs an instance of the DoubleSolenoid48 class.
 * Passes the module number (CAN ID) PCM Channel (0-7) to the two
 * member solenoid object constructors.
 *
 * @param aModule The CAN module ID of the first side of the double solenoid
 * @param aChannel The channel of the first side of the double solenoid
 * @param bModule The CAN module UD of the second side of the double solenoid
 * @param bChannel The channel of the second side of the double solenoid
 */
DoubleSolenoid48::DoubleSolenoid48(uint8_t aModule, uint32_t aChannel, uint8_t bModule, uint32_t bChannel)
{
	aSolenoid = new frc::Solenoid(aModule, aChannel);
	bSolenoid = new frc::Solenoid(bModule, bChannel);
}

/**
 * Cleans up the two solenoid member objects.
 */
DoubleSolenoid48::~DoubleSolenoid48()
{
	delete aSolenoid;
	delete bSolenoid;
}

/**
 * Forwards the set request to the solenoid member objects.
 * 
 * @param state The state to set the double solenoid to
 */
void DoubleSolenoid48::Set(DoubleSolenoid48State state)
{
	switch (state)
	{
		case OnA:
			aSolenoid->Set(true);
			bSolenoid->Set(false);
			break;
		case OnB:
			aSolenoid->Set(false);
			bSolenoid->Set(true);
			break;
		default:
			aSolenoid->Set(false);
			bSolenoid->Set(false);
	}
}

/**
 * Retrieves the state of the double solenoid.
 */
DoubleSolenoid48::DoubleSolenoid48State DoubleSolenoid48::Get()
{
	if (aSolenoid->Get())
		return OnA;
	if (bSolenoid->Get())
		return OnB;
	return kOff;
}

