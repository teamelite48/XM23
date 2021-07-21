#ifndef XM23LIB_H_
#define XM23LIB_H_

/**
 * Class of static methods for common operations.
 */ 
class XM23Lib
{
public:
	static float limitOutput(float f);
	static float signSquare(float f);
	static float signCube(float f);
	static float sq(float x);
	static float rangeLimit(float x, float ubound, float lbound);

private:
	// Ensure this class can't be instantiated.
	XM23Lib() {}
};

#endif // XM23LIB_H_
