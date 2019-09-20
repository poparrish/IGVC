#include <iostream>
#include <cstdlib>
#include "Vec2d.h"
#include <math.h>

//macros
#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

//construcor
Vec2d::Vec2d(float angle, int mag)
{
	SetVec2d(angle,mag);
}

//vec2d member functions
void Vec2d::SetVec2d(float angle, int mag)
{
	m_angle = angle;
	m_mag = mag;
	m_x = cos(degToRad(angle))*mag;
	m_y = sin(degToRad(angle))*mag;
}
