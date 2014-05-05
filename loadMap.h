#pragma once
#include <string>
#include <iostream>
#include <stdexcept>
#include "chai3d.h"
#include "Constants.h"
#include <unordered_set>


using std::string; using std::cerr; using std::endl; using std::pair;

#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str()) // convert to resource path
class LoadMap
{
public:
	LoadMap()
	{

	}
	LoadMap(char* argv[])
	{
		resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);
	}

	void initialize(char* argv[])
	{
		resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);
	}
	/*
	cColorb does not overload the == operator, so this function checks
	if two colours are the same.
	*/
	bool color_equal(const cColorb& c1, const cColorb& c2)
	{
		return (c1.getR() == c2.getR() && c1.getG() == c2.getG() && c1.getB() == c2.getB());
	}

	/*
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
				if (COLOUR_TO_DIR[color] == NONE) // Only add fields with a direction
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
				fields.push_back(areas.back().create_field(imageWidth, imageHeight, size, offsetU, offsetV, scaleFactor, COLOUR_TO_DIR[color]));
			}
		}
		return fields;
	}
	
	/*
		Load the walls
	*/
	int loadWalls(cWorld* world, cMesh * obj, double proxyRadius, vector<Field>& fields)
	{
		cerr << "Loading walls" << endl;
		return loadHeightMap(world, obj, proxyRadius, false, true,false,fields);
	}

	/*
		Load the roof
	*/
	int loadRoof(cWorld* world, cMesh * obj, double proxyRadius)
	{
		cerr << "Loading roof" << endl;
		return loadHeightMap(world, obj, proxyRadius, true, false,false);
	}

	/*
		Load the switchwalls
	*/
	int loadSwitchWalls(cWorld* world, cMesh * obj, double proxyRadius)
	{
		cerr << "Loading switchwalls" << endl;
		return loadHeightMap(world, obj, proxyRadius, false, false,true);
	}

private:
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

	bool isSwitch(cTexture2D* newTexture, int u, int v)
	{
		cColorb color = newTexture->m_image.getPixelColor(u, v);
		return (color.getR() == 128 && color.getG() == 128 && color.getB() == 128);
	}

	bool isWall(cTexture2D* newTexture, int u, int v)
	{
		cColorb color = newTexture->m_image.getPixelColor(u, v);
		return (color.getR() == 0 && color.getG() == 0 && color.getB() == 255);
	}



	/*
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
		
		vector < vector< int >> loadedPixels(imageHeight,vector<int>(imageWidth,-1));
		vector<vector<int>> triangles;

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
		bool up = false, down = false, right = false, left = false;

		for (int v = 0; v < imageHeight; v++) // For each pixel of the image, create a vertex
		{
			for (int u = 0; u < imageWidth; u++)
			{
				double px, py;
				cColorb color = newTexture->m_image.getPixelColor(u, v);
				double height = 0;
				//if (color.getR() == 0 && color.getB() == 0 && color.getG() == 0) // No need to create triangles for black areas
				//	continue;

				if (loadRoof) // create the roof 
				{
					height = 0.1;
					px = imageSize * (double) u - offsetU - 0.1;
					py = imageSize * (double) v - offsetV;
				}
				else if (!loadSwitchWalls && isWall(newTexture,u,v)) // Walls are blue
				{					
					height = 0.1;
				}
				else if (loadSwitchWalls && isSwitch(newTexture,u,v))
				{
					if (!left && u != 0 && v != 0)
					{
						if (!isSwitch(newTexture, u - 1, v))
						{
							left = true;
							auto p = loadSide(loadedPixels, newTexture, obj, u - 1, v, 0, 1);
// 							int i = u - 1;
// 							int j = v;
// 							while (!isSwitch(newTexture, i, j) && isSwitch(newTexture,i+1,j)) //left
// 							{
// 								px = imageSize * (double) i - offsetU;
// 								py = imageSize * (double) j - offsetV;
// 								cerr << "left: " << i << " " << j << endl;
// 								unsigned int leftindex = obj->newVertex(px, py, 0); // create new vertex
// 								cVertex* leftvertex = obj->getVertex(leftindex);
// 								leftvertex->setTexCoord(double(i) / double(imageWidth), double(j) / double(imageHeight));
// 								loadedPixels[i][j] = leftindex;
// 								j++;
// 							}
							int i;
							int j;
							if (!isSwitch(newTexture, p.first+1, p.second) && !up) //up
							{
								up = true;
								i = u;
								j = p.second;
								while (!isSwitch(newTexture, i, j) && isSwitch(newTexture, i, j - 1))
								{
									px = imageSize * (double) i - offsetU;
									py = imageSize * (double) j - offsetV;
									cerr << "up: " << i << " " << j << endl;
									unsigned int upindex = obj->newVertex(px, py, 0); // create new vertex
									cVertex* upvertex = obj->getVertex(upindex);
									upvertex->setTexCoord(double(i) / double(imageWidth), double(j) / double(imageHeight));
									loadedPixels[i][j] = upindex;
									i++;
								}
							}
							px = imageSize * (double) i - offsetU;
							py = imageSize * (double) j - offsetV;
							cerr << "special: " << i << " " << j;
							unsigned int tmp = obj->newVertex(px, py, 0); // create new vertex
							cVertex* tmpv = obj->getVertex(tmp);
							tmpv->setTexCoord(double(i) / double(imageWidth), double(j) / double(imageHeight));
							loadedPixels[i][j] = tmp;
						}
					}
					if (!down && u != imageWidth - 1 && v != 0)
					{
						if (!isSwitch(newTexture, u, v-1))
						{
							down = true;
							int i = u;
							int j = v-1;
							while (!isSwitch(newTexture, i, j) && isSwitch(newTexture, i, j+1)) //down
							{
								px = imageSize * (double) i - offsetU;
								py = imageSize * (double) j - offsetV;
								cerr << "down: " << i << " " << j << endl;
								unsigned int downindex = obj->newVertex(px, py, 0); // create new vertex
								cVertex* downvertex = obj->getVertex(downindex);
								downvertex->setTexCoord(double(i) / double(imageWidth), double(j) / double(imageHeight));
								loadedPixels[i][j] = downindex;
								i++;
							}

							j += 1;
							if (!isSwitch(newTexture, i, j))
							{
								
								right = true;
								while (!isSwitch(newTexture, i, j) && isSwitch(newTexture, i - 1, j)) //left
								{
									px = imageSize * (double) i - offsetU;
									py = imageSize * (double) j - offsetV;
									cerr << "right: " << i << " " << j << endl;
									unsigned int rightindex = obj->newVertex(px, py, 0); // create new vertex
									cVertex* rightvertex = obj->getVertex(rightindex);
									rightvertex->setTexCoord(double(i) / double(imageWidth), double(j) / double(imageHeight));
									loadedPixels[i][j] = rightindex;
									j++;
								}
							}
							up = false;
							down = false;
							right = false;
							left = false;
						}
					}

					height = 0.1;
					px = imageSize * (double) u - offsetU;
					py = imageSize * (double) v - offsetV;
					unsigned int index = obj->newVertex(px, py, height); // create new vertex
					cVertex* vertex = obj->getVertex(index);
					vertex->setTexCoord(double(u) / double(imageWidth), double(v) / double(imageHeight));
					loadedPixels[u][v] = index;
				}
				if (!loadSwitchWalls)
				{

					px = imageSize * (double) u - offsetU;
					py = imageSize * (double) v - offsetV;
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

				//cerr << u << " " << v << endl;
				//cerr << index00 << " " << index01 << " " << index10<<" "<<endl;
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

			// We'll center all vertices, then multiply by this amount,
			// to scale to the desired size.
			scaleFactor = MESH_SCALE_SIZE / objectSize;
			initVars = true;
		}
		

		
		obj->scale(scaleFactor);

		// Load all the forcefields when loading the walls
		if (loadForces)
			fields = loadFields(newTexture, scaleFactor);

		obj->computeBoundaryBox(true); // compute size of object again

		// Build a collision-detector for this object, so
		// the proxy will work nicely when haptics are enabled.
		obj->createAABBCollisionDetector(1.01 * proxyRadius, true, false);

		// set size of frame and normals, then render and update
		obj->setFrameSize(0.2, true);
		obj->setNormalsProperties(0.01, cColorf(1.0, 0.0, 0.0, 1.0), true);
		obj->setUseCulling(false);
		obj->computeGlobalPositions();
		obj->setTransparencyLevel(0, true, true);  // Set mesh as invisible
		return EXIT_SUCCESS;
	}

	pair<int,int> loadSide(vector< vector< int >>& loadedPixels, cTexture2D* newTexture, cMesh * obj, int start_x, int start_y, int x_direction, int y_direction)
	{
		int i = start_x;
		int j = start_y;
		double offsetU = 0.5 * (double) imageWidth * imageSize;
		double offsetV = 0.5 * (double) imageHeight * imageSize;

		while (!isSwitch(newTexture, i, j) && isSwitch(newTexture, i + 1, j)) //left
		{
			double px = imageSize * (double) i - offsetU;
			double py = imageSize * (double) j - offsetV;
			
			unsigned int index = obj->newVertex(px, py, 0); // create new vertex
			cVertex* vertex = obj->getVertex(index);
			vertex->setTexCoord(double(i) / double(imageWidth), double(j) / double(imageHeight));
			loadedPixels[i][j] = index;
			i += x_direction;
			j += y_direction;
		}
		return std::make_pair(i, j);
	}
};
