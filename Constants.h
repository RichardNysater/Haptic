/*
	This header contains most of the constants used in the program.
*/
#pragma once
#include "chai3d.h"
#include <iostream>
#include <map>

typedef std::pair<double, double> point;

const int WINDOW_SIZE_W = 800, WINDOW_SIZE_H = 600;// initial size (width/height) in pixels of the display window
const int OPTION_FULLSCREEN = 1, OPTION_WINDOWDISPLAY = 2; // mouse menu options (right button)
const double MESH_SCALE_SIZE = 2.0; // size of map
const int FORCE_AMOUNT = 0;


/*
The available directions for forces.
*/
enum FORCE_DIRECTION
{
	UP,
	DOWN,
	LEFT,
	RIGHT, 
	NONE
};

/*
	Comparator required to sort cColorb inside the map
*/
struct colorcomp {
	bool operator() (const cColorb& lhs, const cColorb& rhs) const{
		if (lhs.getR() < rhs.getR())
			return true;
		else if (lhs.getR() == rhs.getR() && lhs.getG() < rhs.getG())
			return true;
		else if (lhs.getR() == rhs.getR() && lhs.getG() == rhs.getG() &&lhs.getB() < rhs.getB())
				return true;
		else
			return false;
	}
};

// Colours represent force directions, this map keeps track of them
std::map<cColorb, FORCE_DIRECTION, colorcomp> COLOUR_TO_DIR; 

/*
Initializes constants that cannot be initialized on compile time.
Yes, this can be done in c+11 or with boost, but we can use neither.
*/
void initializeConstants()
{
	COLOUR_TO_DIR[cColorb(255, 0, 255)] = UP;  // Pink
	COLOUR_TO_DIR[cColorb(255, 0, 0)] = RIGHT; // Red
	COLOUR_TO_DIR[cColorb(255, 255, 0)] = DOWN; // Yellow
	COLOUR_TO_DIR[cColorb(0, 255, 0)] = LEFT; // Green
	COLOUR_TO_DIR[cColorb(0, 0, 255)] = NONE; // Blue
}


/*
	Handles the force fields on the map.
*/
class Field
{
public:

	/* 
		Sets the 4 points and direction of this rectangular field
	*/
	Field(point upper_left, point upper_right, point lower_left, point lower_right, FORCE_DIRECTION direction)
	{
		ul = upper_left;
		ur = upper_right;
		ll = lower_left;
		lr = lower_right;
		dir = direction;
	}

	/*
		Checks if a tool is inside of this field
	*/
	bool isInside(cGeneric3dofPointer * tool) const
	{
		cVector3d pos = tool->getDeviceGlobalPos();
		return (pos.x >= ul.first && pos.x <= ur.first && pos.y >= ul.second && pos.y <= ll.second);
	}

	/*
		Returns this field's direction
	*/
	FORCE_DIRECTION direction()
	{
		return dir;
	}

	// You can print the field
	friend	std::ostream& operator<<(std::ostream& os, const Field& f);


private:
	std::pair<double, double> ul, ur, ll, lr;
	FORCE_DIRECTION dir;

};

/*
	Prints a field
*/
std::ostream& operator<<(std::ostream& os, const Field& f)
{
	os << "Printing field: " << std::endl;
	os << "Upper left: " << f.ul.first << " " << f.ul.second << std::endl;
	os << "Upper right: " << f.ur.first << " " << f.ur.second << std::endl;
	os << "Lower left: " << f.ll.first << " " << f.ll.second << std::endl;
	os << "Lower right: " << f.lr.first << " " << f.lr.second << std::endl;
	return os;
}

/*
	The pixelarea represents one of the fields in the loaded bitmap.
	This area can be converted into a field.
*/
class PixelArea
{
public:

	/*
		Sets the 4 points of this rectangular area
	*/
	PixelArea(std::pair<int, int> upper_left, std::pair<int, int> upper_right, std::pair<int, int> lower_left, std::pair<int, int> lower_right)
	{
		ul = upper_left;
		ur = upper_right;
		ll = lower_left;
		lr = lower_right;
	}

	/*
		Checks if a coordinate is inside the area
	*/
	bool isInside(int x, int y)
	{
		return(x >= ul.first && x <= ur.first && y >= ul.second && y <= lr.second);
	}

	/*
		Convert this area into a field with a direction.
	*/
	Field create_field(int xSize, int ySize, double size, double offsetX, double offsetY,double scaleFactor, FORCE_DIRECTION dir)
	{
		std::pair<double, double> upper_left, upper_right, lower_left, lower_right;
		upper_left = make_field_pair(ul, size, offsetX, offsetY,scaleFactor);
		upper_right = make_field_pair(ur, size, offsetX, offsetY, scaleFactor);
		lower_left = make_field_pair(ll, size, offsetX, offsetY, scaleFactor);
		lower_right = make_field_pair(lr, size, offsetX, offsetY, scaleFactor);
		return Field(upper_left, upper_right, lower_left, lower_right, dir);
	}

	
private:

	/*
		Scale a pixel coordinate into a real-world coordinate
	*/
	std::pair<double, double> make_field_pair(std::pair<int, int>& in, double size, double offsetX, double offsetY, double scaleFactor)
	{
		return std::make_pair(scaleFactor*(size*(double(in.first)) - offsetX), scaleFactor*((size*double(in.second)) - offsetY));
	}
	std::pair<int, int> ul, ur, ll, lr;
};