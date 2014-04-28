#include "chai3d.h"

const int WINDOW_SIZE_W = 800, WINDOW_SIZE_H = 600;// initial size (width/height) in pixels of the display window
const int OPTION_FULLSCREEN = 1, OPTION_WINDOWDISPLAY = 2; // mouse menu options (right button)
const double MESH_SCALE_SIZE = 2.0; // size of map
const int FORCE_AMOUNT = 4;

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
Handles the forces.
Colours: up = red, down = green, left = yellow, right = pink
*/
class Forces
{
public:

	Forces(int forceAmount)
	{
		force_v.resize(forceAmount);
	}

	cMesh *& operator[](const unsigned int element)
	{
		return force_v[element];
	}

	const cMesh *const & operator[](const unsigned int element) const
	{
		return force_v[element];
	}

	cMesh *& operator()(const unsigned int element)
	{
		return force_v[element];
	}

	const cMesh *const & operator()(const unsigned int element) const
	{
		return force_v[element];
	}

	vector<cMesh *>::iterator begin()
	{
		return force_v.begin();
	}

	vector<cMesh *>::iterator end()
	{
		return force_v.end();
	}

	unsigned int size() const
	{
		return force_v.size();
	}
private:
	std::vector<cMesh*> force_v;
};