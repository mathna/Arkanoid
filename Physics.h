#pragma once

#include "Figure.h" 

namespace MF
{
	typedef struct Border
	{
		double bottom_left_corner_x; 
		double bottom_left_corner_y; 
		double top_right_x; 
		double top_right_y; 
	} Border;

	class Physics : public Figure
	{
	protected:
		int m_previous_time; 
		Border m_border; 
		double m_v;
		double m_alfa_v;
		double m_g; 
		double m_alfa_g;

	public:
		Physics();
		~Physics();
		void Update(int current_time);
		void SetInitialParameters(int current_time, double v, double alfa_v, double g, double alfa_g);
		virtual bool Collision(const Physics& X); 

	protected:
		void SetBorders(double bottom_left_corner_x, double bottom_left_corner_y, double top_right_x, double top_right_y);

	private:
		void Rebound(double alfa_n); 
		bool IsInRectangle(double _x, double _y, const Physics& X);
		double Distance(double _x, double _y, double _bottom_left_corner_x, double _bottom_left_corner_y, double _top_right_x, double _top_right_y);
		double FindNormal(const Physics& X);
}
