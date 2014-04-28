//===========================================================================
/*
This file is part of a haptics course project
and is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License("GPL") version 2
as published by the Free Software Foundation.

\author    Richard Nysater
*/
//===========================================================================

#include <assert.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chai3d.h"
#include "Constants.h"

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------
long long iterations = 0;
FORCE_DIRECTION direction = NONE;
Forces forces(FORCE_AMOUNT);
bool worldTransparent = false, forcesTransparent = true;
int invertation = 1;// -1 if inverted

// Pre-existing variables
cWorld* world; // a world that contains all objects of the virtual environment
cCamera* camera; // a camera that renders the world in a window display
cMesh* solidWorld, *roof; // a mesh object used to create the height map
cLight *light; // a light source to illuminate the objects in the virtual scene
int displayW = 0, displayH = 0; // width and height of the current window display
cHapticDeviceHandler* handler; // a haptic device handler
cGeneric3dofPointer* tool; // a 3D cursor which represents the haptic device 
double proxyRadius;// radius of the tool proxy
bool programRunning = false; // status of the main simulation haptics loop
double cameraAngleH, cameraAngleV, cameraDistance; // camera position and orientation is spherical coordinates
cVector3d cameraPosition;
bool flagCameraInMotion;
int mouseX, mouseY, mouseButton; // mouse position and button status
string resourceRoot; // root resource path
bool programFinished = false; // has exited haptics simulation thread

#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str()) // convert to resource path

// DECLARED FUNCTIONS
void updateCameraPosition(); // update camera settings
void resizeWindow(int w, int h); // callback when the window display is resized
void keySelect(unsigned char key, int x, int y); // callback when a keyboard key is pressed
void mouseClick(int button, int state, int x, int y); // callback to handle mouse click
void mouseMove(int x, int y); // callback to handle mouse motion
void close(void); // function called before exiting the application
void updateGraphics(void); // main graphics callback
void updateHaptics(void); // main haptics loop
int loadHeightMap(cMesh*, int, int, int); // loads a bitmap file and create 3D height map based on pixel color
void glut_start(int argc, char** argv); // Start rendering
const double initializeTools(); // Initialize the tool and haptic devices
void loadWorld(const double); // Load the world

/*
	Loads the world. This means the walls, invisible roof and forces.
*/
void loadWorld(const double stiffnessMax)
{
	solidWorld = new cMesh(world); // create new meshes for the solid world and the forces

	world->addChild(solidWorld);
	roof = new cMesh(world);
	world->addChild(roof);
	for (unsigned int i = 0; i < forces.size(); ++i)
	{
		forces[i] = new cMesh(world);
		world->addChild(forces[i]);
	}

	// load maps
	loadHeightMap(solidWorld, 0, 0, 255); // Solid world is blue
	loadHeightMap(forces[UP], 255, 0, 0); // Up is red
	loadHeightMap(forces[DOWN], 0, 255, 0); // Down is green
	loadHeightMap(forces[LEFT], 255, 255, 0); // Left is yellow
	loadHeightMap(forces[RIGHT], 255, 0, 255); // Right is pink
	loadHeightMap(roof, 255, 255, 255); // Roof special case
	solidWorld->setTransparencyLevel(100, true, true); //Solid world should be opaque
	solidWorld->setUseTexture(false);
	solidWorld->setStiffness(0.5 * stiffnessMax, true);
}

/*
	Initialize the tool and other haptic devices.
*/
const double initializeTools()
{
	handler = new cHapticDeviceHandler(); // create a haptic device handler

	cGenericHapticDevice* hapticDevice; // get access to the first available haptic device
	handler->getDevice(hapticDevice, 0);

	cHapticDeviceInfo info; // retrieve information about the current haptic device
	if (hapticDevice)
	{
		info = hapticDevice->getSpecifications();
	}
	tool = new cGeneric3dofPointer(world); // create a 3D tool and add it to the world
	camera->addChild(tool);// attach tool to camera

	// position tool workspace in front of camera (x-axis of camera reference pointing towards the viewer)
	tool->setPos(-cameraDistance, 0.0, 0.0);
	tool->setHapticDevice(hapticDevice); // connect the haptic device to the tool
	tool->start(); 	// initialize tool by connecting to haptic device
	tool->setWorkspaceRadius(1.0); // map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setRadius(0.03); // define a radius for the tool (graphical display)
	tool->m_deviceSphere->setShowEnabled(false); // hide the device sphere. only show proxy.
	proxyRadius = 0.0; 	// set the physical readius of the proxy.
	tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);
	tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	const double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// define a maximum stiffness that can be handled by the current
	// haptic device. The value is scaled to take into account the
	// workspace scale factor
	const double stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;
	return stiffnessMax;
}

/*
	Initialize the camera and light.
*/
void initializeCamera()
{
	camera = new cCamera(world); // create a camera and insert it into the virtual world
	world->addChild(camera);

	cameraAngleH = 0;
	cameraAngleV = 90;
	cameraDistance = 1.8 * MESH_SCALE_SIZE;
	updateCameraPosition(); // define a default position of the camera (described in spherical coordinates)

	// set the near and far clipping planes of the camera
	// anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	light = new cLight(world); // create a light source and attach it to the camera
	camera->addChild(light);                   // attach light to camera
	light->setEnabled(true);                   // enable light source
	light->setPos(cVector3d(0.0, 0.3, 0.3));  // position the light source
	light->setDir(cVector3d(-1.0, -0.1, -0.1));  // define the direction of the light beam
	light->m_ambient.set(0.5, 0.5, 0.5);
	light->m_diffuse.set(0.8, 0.8, 0.8);
	light->m_specular.set(1.0, 1.0, 1.0);
}

/*
Update the size of the viewport
*/
void resizeWindow(int w, int h)
{
	displayW = w;
	displayH = h;
	glViewport(0, 0, displayW, displayH);
}

/*
When a button is pressed, this function is called.
Inside this function we therefore handle all key-based events.
*/
void keySelect(unsigned char key, int x, int y)
{
	if ((key == 27) || (key == 'x')) // Escape key
	{
		close();
		exit(EXIT_SUCCESS);
	}
	else if (key == '1')
	{
		bool useTexture = solidWorld->getUseTexture();
		solidWorld->setUseTexture(!useTexture);
	}
	else if (key == '2')
	{
		bool useWireMode = solidWorld->getWireMode();
		solidWorld->setWireMode(!useWireMode);
	}
	else if (key == '3')
	{
		// INTENTIONALLY LEFT BLANK
	}
	else if (key == '4')
	{
		invertation *= -1;
	}
	else if (key == '5')
	{
		if (worldTransparent)
		{
			solidWorld->setTransparencyLevel(0, true, true);
		}
		else
		{
			solidWorld->setTransparencyLevel(100, true, true);
		}
		worldTransparent = !worldTransparent;
	}
	else if (key == '6')
	{
		if (forcesTransparent)
		{
			for (int i = 0; i<forces.size(); ++i)
			{
				forces[i]->setTransparencyLevel(0, true, true);
			}
		}
		else
		{
			for (int i = 0; i<forces.size(); ++i)
			{
				forces[i]->setTransparencyLevel(100, true, true);
			}
		}
		forcesTransparent = !forcesTransparent;
	}
}

/*
When the mouse button is pressed, the world can be moved
*/
void mouseClick(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN) // mouse button down
	{
		flagCameraInMotion = true;
		mouseX = x;
		mouseY = y;
		mouseButton = button;
	}
	else if (state == GLUT_UP) // mouse button up
	{
		flagCameraInMotion = false;
	}
}

/*
When the mouse moves, the world should move with it
*/
void mouseMove(int x, int y)
{
	if (flagCameraInMotion)
	{
		if (mouseButton == GLUT_RIGHT_BUTTON)
		{
			cameraDistance = cameraDistance - 0.01 * (y - mouseY);
		}

		else if (mouseButton == GLUT_LEFT_BUTTON)
		{
			cameraAngleH = cameraAngleH - (x - mouseX);
			cameraAngleV = cameraAngleV + (y - mouseY);
		}
	}
	updateCameraPosition();

	mouseX = x;
	mouseY = y;
}

/*
Close the program
*/
void close(void)
{
	programRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!programFinished)
	{
		cSleepMs(100);
	}
	tool->stop();
}

/*
Updates the haptic feedback
*/
void updateHaptics(void)
{
	const int STATE_IDLE = 1;
	const int STATE_MOVE_CAMERA = 2;
	int state = STATE_IDLE;

	cVector3d toolGlobalPos, toolLocalPos, prevToolGlobalPos, prevToolLocalPos; // Tool positions

	while (programRunning)
	{
		world->computeGlobalPositions(true); // compute global reference frames for each object

		tool->updatePose(); // update position, orientation of tool and compute forces
		tool->computeInteractionForces();

		bool userSwitch = tool->getUserSwitch(0);

		toolGlobalPos = tool->getDeviceGlobalPos(); // update tool position
		toolLocalPos = tool->getDeviceLocalPos();

		if ((state == STATE_MOVE_CAMERA) && (!userSwitch)) // Stop moving camera
		{
			state = STATE_IDLE;
			solidWorld->setHapticEnabled(true, true);
			for (unsigned int j = 0; j < forces.size(); ++j)
			{
				forces[j]->setHapticEnabled(true, true);
			}
		}
		else if ((state == STATE_IDLE) && (userSwitch)) // Start moving camera
		{
			state = STATE_MOVE_CAMERA;
			solidWorld->setHapticEnabled(false, true);

			for (unsigned int j = 0; j < forces.size(); ++j)
			{
				forces[j]->setHapticEnabled(false, true);
			}
		}
		else if (state == STATE_MOVE_CAMERA) // Keep moving camera
		{
			cVector3d offset = toolLocalPos - prevToolLocalPos; // compute tool offset
			cameraDistance = cameraDistance - 2 * offset.x; // apply camera motion
			cameraAngleH = cameraAngleH - 40 * offset.y;
			cameraAngleV = cameraAngleV - 40 * offset.z;

			updateCameraPosition();
		}

		for (unsigned int i = 0; i < forces.size(); ++i) // Check if we are affected by a new force
		{
			if (tool->isInContact(forces[i]))
			{
				for (unsigned int j = 0; j < forces.size(); ++j)
				{
					if (j != i)
						forces[j]->setHapticEnabled(true);
				}
				forces[i]->setHapticEnabled(false);
				std::cerr << "contact with " << i << std::endl;
				direction = (FORCE_DIRECTION) i;
			}
		}

		prevToolLocalPos = toolLocalPos;
		prevToolGlobalPos = toolGlobalPos;

		int local_invertation = invertation; // Check if we want to invert the force
		if (local_invertation == 1)
		{
			if (tool->getUserSwitch(0))
			{
				local_invertation = -1;
				std::cerr << " INVERTING " << std::endl;
			}
			else
			{
				local_invertation = 1;
			}
		}

		if (invertation == -1)
			std::cerr << " inverted " << std::endl;
		switch (direction)
		{
		case DOWN:
			//std::cerr << "DOWN " << std::endl;
			//tool->m_lastComputedGlobalForce.add(cVector3d(0, 0, -10*local_invertation));
			tool->m_lastComputedGlobalForce.x = 3; //* local_invertation;
			tool->m_lastComputedGlobalForce.y = 0;
			break;
		case UP:
			//std::cerr << "UP " << std::endl;
			tool->m_lastComputedGlobalForce.x = -3;
			tool->m_lastComputedGlobalForce.y = 0;
			break;
		case LEFT:
			//std::cerr << "LEFT " << std::endl;
			tool->m_lastComputedGlobalForce.x = 0;
			tool->m_lastComputedGlobalForce.y = 3;
			break;
		case RIGHT:
			//std::cerr << "RIGHT " << std::endl;
			tool->m_lastComputedGlobalForce.x = 0;
			tool->m_lastComputedGlobalForce.y = -3;
			break;
		}
		tool->applyForces();


	}
	programFinished = true;

}

/*
Create a new mesh from a map-file.
This is used to create the objects the tool can interact with.

The parameters red, green and blue specify what the colour of a pixel should be
to become part of the mesh.
*/
int loadHeightMap(cMesh * obj, int red, int green, int blue)
{
	cTexture2D* newTexture = new cTexture2D();
	world->addTexture(newTexture);
	std::cerr << resourceRoot << std::endl;
#if defined(_MSVC)
	bool fileload = newTexture->loadFromFile("../../../map.bmp");
#else
	bool fileload = newTexture->loadFromFile(RESOURCE_PATH("images/map.bmp"));
#endif
	std::cerr << fileload << std::endl;
	if (!fileload)
	{
		printf("Error - Texture image failed to load correctly.\n");
		close();
		return EXIT_FAILURE;
	}

	int texSizeU = newTexture->m_image.getWidth();
	int texSizeV = newTexture->m_image.getHeight();

	if ((texSizeU < 1) || (texSizeV < 1))
	{
		return false;
	}

	int largestSide = max(texSizeV, texSizeU); // we look for the largest side

	// The largest side of the map has a length of 1.0
	// we now compute the respective size for 1 pixel of the image in world space.
	double size = 1.0 / (double) largestSide;

	// we will create an triangle based object. For centering puposes we
	// compute an offset for axis X and Y corresponding to the half size
	// of the image map.
	double offsetU = 0.5 * (double) texSizeU * size;
	double offsetV = 0.5 * (double) texSizeV * size;

	for (int v = 0; v < texSizeV; v++) // For each pixel of the image, create a vertex
	{
		for (int u = 0; u < texSizeU; u++)
		{
			double px, py;

			cColorb color = newTexture->m_image.getPixelColor(u, v);
			double height = -0.01;
			// The height should only be set if the pixel matches
			if (color.getB() == blue && color.getR() == red && color.getG() == green)
			{
				height = 0.1;
			}
			else if (blue == 255 && red == 0 && green == 0) // Or if it's the ground
			{
				height = 0;
			}
			if (blue == 255 && red == 255 && green == 255) // compute the position of the vertex
			{
				px = size * (double) u - offsetU - 0.1;
				py = size * (double) v - offsetV;
				height = 0.1;
			}
			else
			{
				px = size * (double) u - offsetU;
				py = size * (double) v - offsetV;
			}
			unsigned int index = obj->newVertex(px, py, height); // create new vertex>
			cVertex* vertex = obj->getVertex(index);

			vertex->setTexCoord(double(u) / double(texSizeU), double(v) / double(texSizeV));
		}
	}

	for (int v = 0; v < (texSizeV - 1); v++) // Create a triangle based map using the above pixels
	{
		for (int u = 0; u < (texSizeU - 1); u++)
		{
			// get the indexing numbers of the next four vertices
			unsigned int index00 = ((v + 0) * texSizeU) + (u + 0);
			unsigned int index01 = ((v + 0) * texSizeU) + (u + 1);
			unsigned int index10 = ((v + 1) * texSizeU) + (u + 0);
			unsigned int index11 = ((v + 1) * texSizeU) + (u + 1);

			// create two new triangles
			obj->newTriangle(index00, index01, index10);
			obj->newTriangle(index10, index01, index11);
		}
	}

	// apply texture, compute normals and size
	obj->setTexture(newTexture);
	obj->setUseTexture(true);
	obj->computeAllNormals(true);
	obj->computeBoundaryBox(true);

	cVector3d min = obj->getBoundaryMin();
	cVector3d max = obj->getBoundaryMax();

	cVector3d span = cSub(max, min); // This is the "size" of the object
	size = cMax(span.x, cMax(span.y, span.z));

	// We'll center all vertices, then multiply by this amount,
	// to scale to the desired size.
	double scaleFactor = MESH_SCALE_SIZE / size;
	obj->scale(scaleFactor);

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

/*
Updates the camera position
*/
void updateCameraPosition()
{
	if (cameraDistance < 0.1) // Check bounds
	{
		cameraDistance = 0.1;
	}

	if (cameraAngleV > 89)
	{
		cameraAngleV = 89;
	}
	else if (cameraAngleV < -89)
	{
		cameraAngleV = -89;
	}

	// compute position of camera in space
	cVector3d pos = cAdd(cameraPosition,
		cVector3d(
		cameraDistance * cCosDeg(cameraAngleH) * cCosDeg(cameraAngleV),
		cameraDistance * cSinDeg(cameraAngleH) * cCosDeg(cameraAngleV),
		cameraDistance * cSinDeg(cameraAngleV)
		)
		);

	cVector3d lookat = cameraPosition; // compute lookat position

	cVector3d up(0.0, 0.0, 1.0); // define role orientation of camera

	camera->set(pos, lookat, up); // set new position to camera

	world->computeGlobalPositions(true); // recompute global positions

	if (tool != NULL) // update tool position
		tool->setPos(-cameraDistance, 0.0, 0.0);
}

/*
Starts the opengl graphics
*/
void glut_start(int argc, char** argv)
{
	glutInit(&argc, argv); // initialize GLUT

	// retrieve the resolution of the computer display and estimate the position
	// of the GLUT window so that it is located at the center of the screen
	int screenW = glutGet(GLUT_SCREEN_WIDTH);
	int screenH = glutGet(GLUT_SCREEN_HEIGHT);
	int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
	int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

	// initialize the OpenGL GLUT window
	glutInitWindowPosition(windowPosX, windowPosY);
	glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(argv[0]);
	glutDisplayFunc(updateGraphics);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMove);
	glutKeyboardFunc(keySelect);
	glutReshapeFunc(resizeWindow);
	glutSetWindowTitle("IDCL - Haptics project");

	// START SIMULATION
	programRunning = true;

	cThread* hapticsThread = new cThread(); // create a thread which starts the main haptics rendering loop
	hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

	glutMainLoop(); 	// start the main graphics rendering loop
	close();
}

/*
Updates the graphics displayed
*/
void updateGraphics(void)
{
	/* update solidWorld normals
	//solidWorld->computeAllNormals(true);
	//for (int i = 0; i<forces.size();++i)
	{
	//    forces[i]->computeAllNormals(true);
	}
	// render world*/
	camera->renderView(displayW, displayH);

	glutSwapBuffers();

	if (programRunning)
	{
		glutPostRedisplay();
	}

	std::cerr << iterations << std::endl;
	iterations++;
}

/*
The main function initializes everything.
*/
int main(int argc, char* argv [])
{
	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);

	world = new cWorld(); // create a new world.

	// set the background color of the environment
	// the color is defined by its (R,G,B) components.
	world->setBackgroundColor(0.1, 0.1, 0.1);

	initializeCamera();
	const double stiffnessMax = initializeTools(); // Initialize the tool and get the max stiffness
	loadWorld(stiffnessMax); // Load the world (walls, roof and forces)
	glut_start(argc, argv); // Start rendering with OpenGL
	return (EXIT_SUCCESS);
}