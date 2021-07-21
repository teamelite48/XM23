 #include <XM23Lib.h>
#include "frc/WPILib.h"

/**
 * Limits a value to the Victor output range of -1.0 to 1.0.
 * 
 * @param f The value to limit
 */
float XM23Lib::limitOutput(float f)
{
	if (f > 1.0)
	{
		return 1.0;
	}
	else if (f < -1.0)
	{
		return -1.0;
	}
	return f;
}

/**
 * Squares a value while preserving sign.
 * 
 * @param f The value to square
 */

float XM23Lib::signSquare(float f)
{	
	if (f < 0)
	{	
		return -1.0 * f * f;
	}
	else
	{
		return f * f;
	}
}

float XM23Lib::signCube(float f)
{
	return f * f * f;
}

float XM23Lib::sq(float x)
{
	return x*x;
}

float XM23Lib::rangeLimit(float x, float ubound, float lbound)
{
	if (x > ubound)
	{
		return ubound;
	}
	else if (x < lbound)
	{
		return lbound;
	}
	
	printf("x returned = %f\n", x);
	
	return x;
}

