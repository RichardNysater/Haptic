/**
	This class handles the loading of resources, mainly the map and the circuit.
	
	@author	Richard Nysater
	@version 1.0 2014-05-07
*/

#pragma once
#include <string>
#include <iostream>
#include <stdexcept>
#include "chai3d.h"
#include "Constants.h"
#include <unordered_set>


using std::string; using std::cerr; using std::endl; using std::pair;

#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str()) // convert to resource path

/**
	The LoadMap class is used to load pictures into objects and overlays.
*/
class LoadMap
{
public:

	/**
		Constructor for LoadMap
	*/
	LoadMap()
	{

	}

	/**
		Constructor for LoadMap
	*/
	LoadMap(char* argv[])
	{
		resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);
	}

	
	/**
		Initialized the resourceRoot to the first argument sent to the program.
	*/
	void initialize(char* argv[])
	{
		resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);
	}

	/**
		Adds a 2d picture as the actual circuit. This picture is the one that is supposed to be displayed,
		while the rest of the objects are invisible.
	*/
	void loadCircuitPicture(cBitmap * circuit, cCamera * camera)
	{
		circuit = new cBitmap();
		
		camera->m_back_2Dscene.addChild(circuit);
		// load a "chai3d" bitmap image file
		bool fileload;
		fileload = circuit->m_image.loadFromFile("circuit.bmp");
		if (!fileload)
		{
			fileload = circuit->m_image.loadFromFile(RESOURCE_PATH("images/circuit.bmp"));
		}
		if (!fileload)
		{
			cerr << "Unable to load circuit. No texture found" << endl;
			return;
		}
			
		// position the logo at the bottom left of the screen (pixel coordinates)
		circuit->setPos(169, 72, 0);

		// scale the logo along its horizontal and vertical axis
		circuit->setZoomHV(2.85, 2.85);

		// here we replace all black pixels (0,0,0) of the logo bitmap
		// with transparent black pixels (0, 0, 0, 0). This allows us to make
		// the background of the logo look transparent.
		
		circuit->m_image.replace(
			cColorb(0, 0, 0),      // original RGB color
			cColorb(0, 0, 0, 0)    // new RGBA color
			);

		// enable transparency
		circuit->enableTransparency(true);

	}

	/*
		cColorb does not overload the == operator, so this function checks
		if two colours are the same.
	*/
	bool color_equal(const cColorb& c1, const cColorb& c2)
	{
		return (c1.getR() == c2.getR() && c1.getG() == c2.getG() && c1.getB() == c2.getB());
	}

	/**
		Loads all the force fields from the bitmap
	*/
	vector<Field> loadFields(cTexture2D * newTexture, double scaleFactor)
	{
		int imageWidth = newTexture->m_image.getWidth();
		int imageHeight = newTexture->m_image.getHeight();

		int largestSide = max(imageHeight, imageWidth); // we look for the largest side

		// The largest side of the map has a length of 1.0
		// we now compute the respective size for 1 pixel of the image in world space.
		double size = 1.0 / (double) largestSide;

		double offsetU = 0.5 * (double) imageWidth * size;
		double offsetV = 0.5 * (double) imageHeight * size;


		std::pair<int, int> upper_left, upper_right, lower_left, lower_right;

		vector<Field> fields;
		vector<PixelArea> areas; // Represents a field in the image.

		for (int u = 0; u < imageWidth; u++) // For each pixel of the image
		{
			for (int v = 0; v < imageHeight; v++)
			{
				bool visited = false;
				for (int index = 0; index < areas.size(); ++index) // Check if we've been at this pixel before
				{
					if (areas[index].isInside(u, v))
					{
						visited = true; // This pixel has already been added
						break;
					}
				}
				if (visited) // Only proceed with non-visited pixels
					continue;

				cColorb color = newTexture->m_image.getPixelColor(u, v);
				if (color.getR() == 255 && color.getB() == 255 && color.getG() == 255)
					continue;
				if (COLOUR_TO_DIR.find(color) == COLOUR_TO_DIR.end()) // Only add fields with a direction
					continue;
	
				int i = u, j = v;
				upper_left = std::make_pair(i, j); // Starting point is upper left

				// While the pixel colour is the same, keep going right
				while (color_equal(color, newTexture->m_image.getPixelColor(i + 1, j)))
				{
					++i;
					if (i == imageWidth - 1)
						break;
				}
				upper_right = std::make_pair(i, j); // Upper right is when colour changed

				// While the pixel colour is the same, keep going down
				while (color_equal(color, newTexture->m_image.getPixelColor(i, j + 1)))
				{
					++j;
					if (j == imageHeight - 1)
						break;
				}
				lower_right = std::make_pair(i, j); // Lower right is when colour changed
				lower_left = std::make_pair(upper_left.first, j); // Lower left is when colour changed both directions

				areas.push_back(PixelArea(upper_left, upper_right, lower_left, lower_right)); // Mark this area as visited

				// Create a new field from this area
				fields.push_back(areas.back().createField(imageWidth, imageHeight, size, offsetU, offsetV, scaleFactor, COLOUR_TO_DIR[color]));
				if (COLOUR_TO_RES.find(color) != COLOUR_TO_RES.end())
				{
					resistors++;
					fields.back().setResistor(true);
				}
				else if (COLOUR_TO_BAT.find(color) != COLOUR_TO_BAT.end())
				{
					fields.back().setBattery(true);
				}
			}
		}
		return fields;
	}
	
	/**
		Load the walls
	*/
	int loadWalls(cWorld* world, cMesh * obj, double proxyRadius, vector<Field>& fields)
	{
		cerr << "Loading walls" << endl;
		return loadHeightMap(world, obj, proxyRadius, false, true,false,fields);
	}

	/**
		Load the roof
	*/
	int loadRoof(cWorld* world, cMesh * obj, double proxyRadius)
	{
		cerr << "Loading roof" << endl;
		return loadHeightMap(world, obj, proxyRadius, true, false,false);
	}

	/**
		Load the switchwalls
	*/
	int loadSwitchWalls(cWorld* world, cMesh * obj, double proxyRadius)
	{
		cerr << "Loading switchwalls" << endl;
		return loadHeightMap(world, obj, proxyRadius, false, false,true);
	}

private:
	enum DIRECTION
	{
		UP,
		DOWN,
		LEFT,
		RIGHT
	};
	bool initVars = false;
	cVector3d span;
	double scaleFactor;
	string resourceRoot; // root resource path
	int imageWidth;
	int imageHeight;
	double offsetU;
	double offsetV;
	double imageSize;
	double objectSize;

	/**
		Returns true if a pixel is part of a switch wall
	*/
	bool isSwitch(cTexture2D* newTexture, int u, int v)
	{
		cColorb color = newTexture->m_image.getPixelColor(u, v);
		return (color.getR() == 128 && color.getG() == 128 && color.getB() == 128);
	}

	/**
		Returns true if a pixel is part of a wall
	*/
	bool isWall(cTexture2D* newTexture, int u, int v)
	{
		cColorb color = newTexture->m_image.getPixelColor(u, v);
		return (color.getR() == 0 && color.getG() == 0 && color.getB() == 255);
	}

	/**
		Create a new mesh from a map-file.
		This is used to create the objects the tool can interact with.
	*/
	int loadHeightMap(cWorld* world, cMesh * obj, double proxyRadius, bool loadRoof, bool loadForces, bool loadSwitchWalls, vector<Field>& fields = vector<Field>())
	{
		cTexture2D* newTexture = new cTexture2D();
		world->addTexture(newTexture);
		#if defined(_MSVC)
			bool fileload = newTexture->loadFromFile("map.bmp");
		#else
			bool fileload = newTexture->loadFromFile(RESOURCE_PATH("images/map.bmp"));
		#endif
		if (!fileload)
		{
			printf("Error - Texture image failed to load correctly.\n");
			return EXIT_FAILURE;
		}
		
		if (!initVars)
		{
			imageWidth = newTexture->m_image.getWidth();
			imageHeight = newTexture->m_image.getHeight();
		}
		
		vector< vector<int> > loadedPixels(imageHeight,vector<int>(imageWidth,-1));
		vector< vector<int> > triangles;

		if ((imageWidth < 1) || (imageHeight < 1)) // Image too small
		{
			cerr << "Image is too small" << endl;
			return false;
		}
		// we will create an triangle based object. For centering puposes we
		// compute an offset for axis X and Y corresponding to the half size
		// of the image map.
		if (!initVars)
		{
			int largestSide = max(imageHeight, imageWidth); // we look for the largest side

			// The largest side of the map has a length of 1.0
			// we now compute the respective size for 1 pixel of the image in world space.
			imageSize = 1.0 / (double) largestSide;

			offsetU = 0.5 * (double) imageWidth * imageSize;
			offsetV = 0.5 * (double) imageHeight * imageSize;
		}

		for (int v = 0; v < imageHeight; v++) // For each pixel of the image, create a vertex
		{
			for (int u = 0; u < imageWidth; u++)
			{
				double px, py;
				cColorb color = newTexture->m_image.getPixelColor(u, v);
				double height = 0;

				px = imageSize * (double) u - offsetU;
				py = imageSize * (double) v - offsetV;
				bool load = false;
				if (loadRoof) // create roof 
				{
					height = 0.1;
					px = imageSize * (double) u - offsetU;
					py = imageSize * (double) v - offsetV;
					load = true;
				}
				else if (!loadSwitchWalls && isWall(newTexture,u,v)) // create walls
				{					
					height = 0.1;
					load = true;
				}
				else if (loadSwitchWalls && isSwitch(newTexture,u,v)) //create switchwalls
				{
					if (!isSwitch(newTexture, u - 1, v) && loadedPixels[u][v] == -1) // Load the pixels next to the switchwalls
					{
						auto p = loadSide(loadedPixels, newTexture, obj, u, v, DIRECTION::LEFT);
						p = loadSide(loadedPixels, newTexture, obj, u, p.second, DIRECTION::UP);
						p = loadSide(loadedPixels, newTexture, obj, u, v, DIRECTION::DOWN);
						p = loadSide(loadedPixels, newTexture, obj, p.first, p.second, DIRECTION::RIGHT);
					}
					height = 0.1;
					load = true;
				}

				if (load || !loadSwitchWalls)
				{
					unsigned int index = obj->newVertex(px, py, height); // create new vertex
					cVertex* vertex = obj->getVertex(index);
					vertex->setTexCoord(double(u) / double(imageWidth), double(v) / double(imageHeight));
					loadedPixels[u][v] = index;
				}
			}
		}

		for (int v = 0; v < (imageHeight - 1); v++) // Create a triangle based map using the above pixels
		{
			for (int u = 0; u < (imageWidth - 1); u++)
			{
				cColorb color = newTexture->m_image.getPixelColor(u, v);
				if (color.getR() == 0 && color.getB() == 0 && color.getG() == 0) // No need to create triangles for black areas
					continue;

				// get the indexing numbers of the next four vertices
				int index00 = loadedPixels[u][v];
				int index01 = loadedPixels[u + 1][v];
				int index10 = loadedPixels[u][v + 1];
				int index11 = loadedPixels[u + 1][v + 1];

				if (index00 != -1 && index01 != -1 && index10 != -1 && index11 != -1)
				{
					obj->newTriangle(index00, index01, index10);
					obj->newTriangle(index10, index01, index11);
				}
			}
		}

		// apply texture, compute normals and size
		obj->setTexture(newTexture);
		obj->setUseTexture(true);
		obj->computeAllNormals(true);
		obj->computeBoundaryBox(true);

		if (!initVars)
		{
			cVector3d min = obj->getBoundaryMin();
			cVector3d max = obj->getBoundaryMax();

			cVector3d span = cSub(max, min); // This is the "size" of the object
			objectSize = cMax(span.x, cMax(span.y, span.z));

			scaleFactor = MESH_SCALE_SIZE / objectSize;
			initVars = true;
		}
		
		obj->scale(scaleFactor);

		if (loadForces)
			fields = loadFields(newTexture, scaleFactor);

		obj->computeBoundaryBox(true);
		obj->createAABBCollisionDetector(1.01 * proxyRadius, true, false);

		// set size of frame and normals, then render and update
		obj->setFrameSize(0.2, true);
		obj->setNormalsProperties(0.01, cColorf(1.0, 0.0, 0.0, 1.0), true);
		obj->setUseCulling(false);
		obj->computeGlobalPositions();
		obj->setTransparencyLevel(0, true, true);  // Objects should be invisible
		return EXIT_SUCCESS;
	}

	/**
		Loads the pixels along one of the sides of an area.
	*/
	pair<int,int> loadSide(vector< vector<int> >& loadedPixels, cTexture2D* newTexture, cMesh * obj, int startX, int startY, DIRECTION dir)
	{
		int xDirection = 0, yDirection = 0, lookX = 0, lookY = 0;

		double offsetU = 0.5 * (double) imageWidth * imageSize;
		double offsetV = 0.5 * (double) imageHeight * imageSize;

		switch (dir)
		{
		case UP:
			startX++;
			xDirection = 1;
			lookY = -1;
			break;
		case DOWN:
			startY--;
			xDirection = 1;
			lookY = +1;
			break;
		case LEFT:
			startX--;
			yDirection = 1;
			lookX = 1;
			break;
		case RIGHT:
			startY++;
			yDirection = 1;
			lookX = -1;
			break;
		}
		int curX = startX;
		int curY = startY;

		while (!isSwitch(newTexture, curX, curY) && isSwitch(newTexture, curX + lookX, curY+lookY) && loadedPixels[curX][curY] == -1) //left
		{
			double px = imageSize * (double) curX - offsetU;
			double py = imageSize * (double) curY - offsetV;
			unsigned int vertexIndex = obj->newVertex(px, py, 0); // create new vertex
			cVertex* vertex = obj->getVertex(vertexIndex);
			vertex->setTexCoord(double(curX) / double(imageWidth), double(curY) / double(imageHeight));
			loadedPixels[curX][curY] = vertexIndex;
			curX += xDirection;
			curY += yDirection;
		}

		if (loadedPixels[curX][curY] == -1) // Pixel not yet loaded
		{
			double px = imageSize * (double) curX - offsetU;
			double py = imageSize * (double) curY - offsetV;
			unsigned int vertexIndex = obj->newVertex(px, py, 0); // create new vertex
			cVertex* vertex = obj->getVertex(vertexIndex);
			vertex->setTexCoord(double(curX) / double(imageWidth), double(curY) / double(imageHeight));
			loadedPixels[curX][curY] = vertexIndex;
		}

		return std::make_pair(curX, curY);
	}
};
