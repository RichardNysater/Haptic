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
	Forces()
	{
		force_v.resize(4);
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
	std::vector<cMesh*> force_v;
private:

};