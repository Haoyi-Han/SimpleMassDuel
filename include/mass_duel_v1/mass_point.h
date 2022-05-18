#ifndef MASS_POINT_H
#define MASS_POINT_H

struct Point2D
{
	double x;
	double y;
	double v;
	double theta;
	bool operator == (Point2D& p)
	{
		return ((x == p.x) && (y == p.y) && (v == p.v) && (theta == p.theta));
	}
};

// functions for one Point2D object
double getVX(Point2D p);
double getVY(Point2D p);
void changePos(Point2D p, double nx, double ny);
std::ostream& operator<<(std::ostream& os, const Point2D& p);
std::string pToString(Point2D p);

// functions for multiple Point2D objects
double calcDistPoints(Point2D pA, Point2D pB);
std::array <double, 3> calcLineEqCoeff (Point2D pA, Point2D pB);
double CalcDistPointLine(Point2D p, std::array <double, 3> l);


#endif // MASS_POINT_H