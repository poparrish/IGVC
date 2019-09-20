#ifndef VEC2D_H
#define VEC2D_H

class Vec2d
{
private:
	float m_angle;
	int m_mag;
	float m_x;
	float m_y;

public:

	Vec2d(float angle,int mag);

	//set
	void SetVec2d(float angle, int mag);

	//get
	float GetAngle()const{return m_angle;}
	int GetMag()const{return m_mag;}
	float GetX()const{return m_x;}
	float GetY()const{return m_y;}
}; 
#endif