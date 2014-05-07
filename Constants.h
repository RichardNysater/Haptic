/**
	This header contains most of the constants used in the program.
	@author	Richard Nysäter
	@version 1.0 2014-05-07
*/
#pragma once
#include "chai3d.h"
#include <iostream>
#include <map>

using std::cerr; using std::endl;

typedef std::pair<double, double> point;

int resistors = 0;
const int START_VELOCITY = 5;
const int WINDOW_SIZE_W = 800, WINDOW_SIZE_H = 600;// initial size (width/height) in pixels of the display window
const int OPTION_FULLSCREEN = 1, OPTION_WINDOWDISPLAY = 2; // mouse menu options (right button)
const double MESH_SCALE_SIZE = 2.0; // size of map


/**
	The available directions for forces.
*/
enum FORCE_DIRECTION
{
	UP,
	DOWN,
	LEFT,
	RIGHT
};

/**
	The available directions for resistors.
*/
enum RESISTOR
{
	UP_RES,
	DOWN_RES,
	LEFT_RES,
	RIGHT_RES
};

/**
	The available directions for batteries.
*/
enum BATTERY
{
	BATTERY_UP,
	BATTERY_DOWN,
	BATTERY_LEFT,
	BATTERY_RIGHT
};

/**
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

// Colours represent different objects, these maps keeps track of them
std::map<cColorb, FORCE_DIRECTION, colorcomp> COLOUR_TO_DIR; 
std::map<cColorb, RESISTOR, colorcomp> COLOUR_TO_RES;
std::map<cColorb, BATTERY, colorcomp> COLOUR_TO_BAT;

/**
	Initializes constants that cannot be initialized on compile time.
	Yes, this can be done in c+11 or with boost, but we can use neither.
*/
void initializeConstants()
{

	// Direction of forces
	COLOUR_TO_DIR[cColorb(255, 0, 255)] = UP;  // Pink
	COLOUR_TO_DIR[cColorb(255, 0, 0)] = RIGHT; // Red
	COLOUR_TO_DIR[cColorb(255, 255, 0)] = DOWN; // Yellow
	COLOUR_TO_DIR[cColorb(0, 255, 0)] = LEFT; // Green

	COLOUR_TO_DIR[cColorb(128, 0, 128)] = UP;  // Pink resistor
	COLOUR_TO_DIR[cColorb(128, 0, 0)] = RIGHT; // Red resistor
	COLOUR_TO_DIR[cColorb(128, 128, 0)] = DOWN; // Yellow resistor
	COLOUR_TO_DIR[cColorb(0, 128, 0)] = LEFT; // Green resistor

	COLOUR_TO_DIR[cColorb(255, 192, 255)] = UP;  // Pink battery
	COLOUR_TO_DIR[cColorb(255, 192, 192)] = RIGHT; // Red battery
	COLOUR_TO_DIR[cColorb(255, 255, 192)] = DOWN; // Yellow battery
	COLOUR_TO_DIR[cColorb(192, 255, 192)] = LEFT; // Green battery

	// Resistors
	COLOUR_TO_RES[cColorb(128, 0, 128)] = UP_RES;  // Pink resistor
	COLOUR_TO_RES[cColorb(128, 0, 0)] = RIGHT_RES; // Red resistor
	COLOUR_TO_RES[cColorb(128, 128, 0)] = DOWN_RES; // Yellow resistor
	COLOUR_TO_RES[cColorb(0, 128, 0)] = LEFT_RES; // Green resistor
			  
	// Batteries
	COLOUR_TO_BAT[cColorb(255, 192, 255)] = BATTERY_UP; // Battery pink
	COLOUR_TO_BAT[cColorb(255, 192, 192)] = BATTERY_RIGHT; // Battery red
	COLOUR_TO_BAT[cColorb(255, 255, 192)] = BATTERY_DOWN; // Battery yellow
	COLOUR_TO_BAT[cColorb(192, 255, 192)] = BATTERY_LEFT; // Battery green
}


/**
	Represents the fields on the map.
*/
class Field
{
public:

	/**
		Sets the 4 points and direction of this rectangular field
	*/
	Field(point upper_left, point upper_right, point lower_left, point lower_right, FORCE_DIRECTION direction)
	{
		upperLeft = upper_left;
		upperRight = upper_right;
		lowerLeft = lower_left;
		lowerRight = lower_right;
		dir = direction;
		fieldXSize = abs(upperLeft.first - upperRight.first);
		fieldYSize = abs(upperLeft.second - lowerRight.second);
		isBattery = false;
		isResistor = false;
	}

	/**
		Set a new velocity if this field is a battery or resistor
	*/
	void setVelocity(cGeneric3dofPointer* tool, double& velocity)
	{
		if (isBattery)
		{
			double distance = distanceFromStart(tool);
			velocity = distance*START_VELOCITY;
			cerr << distance << " inside battery. Setting velocity to: " << velocity << endl;
		}
		else if (isResistor)
		{
			double distance = distanceFromStart(tool);
			velocity = START_VELOCITY - distance*START_VELOCITY;
			cerr << distance << " inside resistor. Setting velocity to: " << velocity << endl;
		}
	}
	
	/**
		Returns true if this field is a battery
	*/
	bool battery()
	{
		return isBattery;
	}

	/**
		Sets this field to be a battery
	*/
	void setBattery(bool i)
	{
		isBattery = i;
	}

	/**
		Returns true if this field is a resistor
	*/
	bool resistor()
	{
		return isResistor;
	}

	/**
		Sets this field to be a resistor
	*/
	void setResistor(bool i)
	{
		isResistor = i;
	}

	/**
		Checks if a tool is inside of this field
	*/
	bool isInside(cGeneric3dofPointer * tool) const
	{
		cVector3d pos = tool->getProxyGlobalPos();
		return (pos.x >= upperLeft.first && pos.x <= upperRight.first && pos.y >= upperLeft.second && pos.y <= lowerLeft.second);
	}

	/**
		Returns this field's direction
	*/
	FORCE_DIRECTION direction()
	{
		return dir;
	}

	// You can print the field
	friend	std::ostream& operator<<(std::ostream& os, const Field& f);


private:
	std::pair<double, double> upperLeft, upperRight, lowerLeft, lowerRight;
	FORCE_DIRECTION dir;
	bool isBattery, isResistor;
	double fieldXSize, fieldYSize;

	/**
		Returns, in percent from 0 to 1, how close the tool is the start of the field, according to
		the circuit's current. 
	*/
	double distanceFromStart(cGeneric3dofPointer* tool)
	{
		switch (dir)
		{
		case UP:
			return 1-(std::abs(tool->getProxyGlobalPos().x - upperLeft.first)) / fieldXSize;
			break;
		case DOWN:
			return (std::abs(tool->getProxyGlobalPos().x - lowerLeft.first)) / fieldXSize;
			break;
		case LEFT:
			return 1-(std::abs(tool->getProxyGlobalPos().y - upperLeft.second)) / fieldYSize;
			break;
		case RIGHT:
			return 1-(std::abs(tool->getProxyGlobalPos().y - lowerLeft.second)) / fieldYSize;
			break;
		default:
			throw std::invalid_argument("Field does not have correct values");
			break;
		}
		
	}
};

/**
	Prints a field
*/
std::ostream& operator<<(std::ostream& os, const Field& f)
{
	os << "Printing field: " << std::endl;
	os << "Upper left: " << f.upperLeft.first << " " << f.upperLeft.second << std::endl;
	os << "Upper right: " << f.upperRight.first << " " << f.upperRight.second << std::endl;
	os << "Lower left: " << f.lowerLeft.first << " " << f.lowerLeft.second << std::endl;
	os << "Lower right: " << f.lowerRight.first << " " << f.lowerRight.second << std::endl;
	return os;
}

/**
	The pixelarea represents one of the areas in the loaded bitmap.
	This class is used to convert an area on the bitmap into a field in the program.
*/
class PixelArea
{
public:

	/**
		Sets the 4 points of this rectangular area
	*/
	PixelArea(std::pair<int, int> upper_left, std::pair<int, int> upper_right, std::pair<int, int> lower_left, std::pair<int, int> lower_right)
	{
		upperLeft = upper_left;
		upperRight = upper_right;
		lowerLeft = lower_left;
		lowerRight = lower_right;
	}

	/**
		Checks if a coordinate is inside the area
	*/
	bool isInside(int x, int y)
	{
		return(x >= upperLeft.first && x <= upperRight.first && y >= upperLeft.second && y <= lowerRight.second);
	}

	/**
		Convert this area into a field with a direction.
	*/
	Field createField(int xSize, int ySize, double size, double offsetX, double offsetY,double scaleFactor, FORCE_DIRECTION dir)
	{
		std::pair<double, double> upper_left, upper_right, lower_left, lower_right;
		upper_left = makeFieldPair(upperLeft, size, offsetX, offsetY,scaleFactor);
		upper_right = makeFieldPair(upperRight, size, offsetX, offsetY, scaleFactor);
		lower_left = makeFieldPair(lowerLeft, size, offsetX, offsetY, scaleFactor);
		lower_right = makeFieldPair(lowerRight, size, offsetX, offsetY, scaleFactor);
		return Field(upper_left, upper_right, lower_left, lower_right, dir);
	}

	
private:
	std::pair<int, int> upperLeft, upperRight, lowerLeft, lowerRight;

	/**
		Scale a pixel coordinate into a real-world coordinates
	*/
	std::pair<double, double> makeFieldPair(std::pair<int, int>& in, double size, double offsetX, double offsetY, double scaleFactor)
	{
		return std::make_pair(scaleFactor*(size*(double(in.first)) - offsetX), scaleFactor*((size*double(in.second)) - offsetY));
	}
	
};