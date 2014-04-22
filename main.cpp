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
#include "../Constants.h"

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W = 1680, WINDOW_SIZE_H = 1050;

// mouse menu options (right button)
const int OPTION_FULLSCREEN = 1, OPTION_WINDOWDISPLAY = 2;

// size of map
const double MESH_SCALE_SIZE = 2.0;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------
FORCE_DIRECTION direction = NONE;

Forces forces(4);

bool worldTransparent = false, forcesTransparent = true;

// -1 if inverted
int invertation = 1;

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a mesh object used to create the height map
cMesh* solidWorld;

// a light source to illuminate the objects in the virtual scene
cLight *light;

// width and height of the current window display
int displayW = 0, displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a 3D cursor which represents the haptic device
cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

// status of the main simulation haptics loop
bool programRunning = false;

// update camera settings
void updateCameraPosition();

// camera position and orientation is spherical coordinates
double cameraAngleH, cameraAngleV, cameraDistance;

cVector3d cameraPosition;

bool flagCameraInMotion;

// mouse position and button status
int mouseX, mouseY, mouseButton;

// root resource path
string resourceRoot;

// has exited haptics simulation thread
bool programFinished = false;

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to handle mouse click
void mouseClick(int button, int state, int x, int y);

// callback to handle mouse motion
void mouseMove(int x, int y);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// loads a bitmap file and create 3D height map based on pixel color
int loadHeightMap(cMesh* , int , int, int);

/*
	The main function initializes everything.
*/
int main(int argc, char* argv[])
{
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------
	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);


	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
	//-----------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	// the color is defined by its (R,G,B) components.
	world->setBackgroundColor(0.1, 0.1, 0.1);

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// define a default position of the camera (described in spherical coordinates)
	cameraAngleH = 0;
	cameraAngleV = 45;
	cameraDistance = 1.8 * MESH_SCALE_SIZE;
	updateCameraPosition();

	// set the near and far clipping planes of the camera
	// anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// create a light source and attach it to the camera
	light = new cLight(world);
	camera->addChild(light);                   // attach light to camera
	light->setEnabled(true);                   // enable light source
	light->setPos(cVector3d(0.0, 0.3, 0.3));  // position the light source
	light->setDir(cVector3d(-1.0, -0.1, -0.1));  // define the direction of the light beam
	light->m_ambient.set(0.5, 0.5, 0.5);
	light->m_diffuse.set(0.8, 0.8, 0.8);
	light->m_specular.set(1.0, 1.0, 1.0);

	//-----------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	cGenericHapticDevice* hapticDevice;
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	cHapticDeviceInfo info;
	if (hapticDevice)
	{
		info = hapticDevice->getSpecifications();
	}

	// create a 3D tool and add it to the world
	tool = new cGeneric3dofPointer(world);

	// attach tool to camera
	camera->addChild(tool);

	// position tool workspace in front of camera (x-axis of camera reference pointing towards the viewer)
	tool->setPos(-cameraDistance, 0.0, 0.0);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// initialize tool by connecting to haptic device
	tool->start();

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.0);

	// define a radius for the tool (graphical display)
	tool->setRadius(0.03);

	// hide the device sphere. only show proxy.
	tool->m_deviceSphere->setShowEnabled(false);

	// set the physical readius of the proxy.
	proxyRadius = 0.0;
	tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);
	tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;


	//-----------------------------------------------------------------------
	// COMPOSE THE VIRTUAL SCENE
	//-----------------------------------------------------------------------

	// create new meshes for the solid world and the forces
	solidWorld = new cMesh(world);
	world->addChild(solidWorld);
	for (unsigned int i = 0; i < forces.size();++i)
	{
		forces[i] = new cMesh(world);
		world->addChild(forces[i]);
	}

	// load maps
	loadHeightMap(solidWorld,0,0,255); // Solid world is blue
	loadHeightMap(forces[UP], 255, 0, 0); // Up is red
	loadHeightMap(forces[DOWN], 0, 255, 0); // Down is green
	loadHeightMap(forces[LEFT], 255, 255, 0); // Left is yellow
	loadHeightMap(forces[RIGHT], 255, 0, 255); // Right is pink
	solidWorld->setTransparencyLevel(100, true, true); //Solid world should be opaque
	solidWorld->setUseTexture(false);

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// define a maximum stiffness that can be handled by the current
	// haptic device. The value is scaled to take into account the
	// workspace scale factor
	double stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;
	solidWorld->setStiffness(0.5 * stiffnessMax, true);

	//-----------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//-----------------------------------------------------------------------

	// initialize GLUT
	glutInit(&argc, argv);

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


	//-----------------------------------------------------------------------
	// START SIMULATION
	//-----------------------------------------------------------------------
	programRunning = true;

	// create a thread which starts the main haptics rendering loop
	cThread* hapticsThread = new cThread();
	hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

	// start the main graphics rendering loop
	glutMainLoop();

	close();
	return (EXIT_SUCCESS);
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
			for (auto i : forces)
			{
				i->setTransparencyLevel(0, true, true);
			}
		}
		else
		{
			for (auto i : forces)
			{
				i->setTransparencyLevel(100, true, true);
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
	Updates the graphics displayed
*/
void updateGraphics(void)
{
	// update solidWorld normals
	solidWorld->computeAllNormals(true);
	for (auto i : forces)
	{
		i->computeAllNormals(true);
	}

	// render world
	camera->renderView(displayW, displayH);

	glutSwapBuffers();

	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

	if (programRunning)
	{
		glutPostRedisplay();
	}
}

/*
	Updates the haptic feedback
*/
void updateHaptics(void)
{
	const int STATE_IDLE = 1;
	const int STATE_MOVE_CAMERA = 2;
	int state = STATE_IDLE;

	// Tool positions
	cVector3d toolGlobalPos, toolLocalPos, prevToolGlobalPos, prevToolLocalPos;

	while (programRunning)
	{
		// compute global reference frames for each object
		world->computeGlobalPositions(true);

		// update position, orientation of tool and compute forces
		tool->updatePose();
		tool->computeInteractionForces();

		bool userSwitch = tool->getUserSwitch(0);

		// update tool position
		toolGlobalPos = tool->getDeviceGlobalPos();
		toolLocalPos = tool->getDeviceLocalPos();

		if ((state == STATE_MOVE_CAMERA) && (!userSwitch)) // Stop moving camera
		{
			state = STATE_IDLE;
			solidWorld->setHapticEnabled(true, true);
			for (auto i : forces)
			{
				i->setHapticEnabled(true, true);
			}
		}
		else if ((state == STATE_IDLE) && (userSwitch)) // Start moving camera
		{
			state = STATE_MOVE_CAMERA;
			solidWorld->setHapticEnabled(false, true);
			for (auto i : forces)
			{
				i->setHapticEnabled(false, true);
			}
		}
		else if (state == STATE_MOVE_CAMERA) // Keep moving camera
		{
			// compute tool offset
			cVector3d offset = toolLocalPos - prevToolLocalPos;

			// apply camera motion
			cameraDistance = cameraDistance - 2 * offset.x;
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
				direction = (FORCE_DIRECTION)i;
			}
		}
	
		prevToolLocalPos = toolLocalPos;
		prevToolGlobalPos = toolGlobalPos;

		// Check if we want to invert the force
		int local_invertation = invertation;
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
			tool->m_lastComputedGlobalForce.z = -10 * local_invertation;
			break;
		case UP:
			//std::cerr << "UP " << std::endl;
			tool->m_lastComputedGlobalForce.z = 10 * local_invertation;
			break;
		case LEFT:
			//std::cerr << "LEFT " << std::endl;
			tool->m_lastComputedGlobalForce.y = -10 * local_invertation;
			break;
		case RIGHT:
			//std::cerr << "RIGHT " << std::endl;
			tool->m_lastComputedGlobalForce.y = 10 * local_invertation;
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

	#if defined(_MSVC)
		bool fileload = newTexture->loadFromFile("../../../bin/resources/images/map.bmp");
	#else
		bool fileload = newTexture->loadFromFile(RESOURCE_PATH("resources/images/map.bmp"));
	#endif

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

	// we look for the largest side
	int largestSide = max(texSizeV, texSizeU);

	// The largest side of the map has a length of 1.0
	// we now compute the respective size for 1 pixel of the image in world space.
	double size = 1.0 / (double) largestSide;

	// we will create an triangle based object. For centering puposes we
	// compute an offset for axis X and Y corresponding to the half size
	// of the image map.
	double offsetU = 0.5 * (double) texSizeU * size;
	double offsetV = 0.5 * (double) texSizeV * size;

	// For each pixel of the image, create a vertex
	for (int v = 0; v < texSizeV; v++)
	{
		for (int u = 0; u < texSizeU; u++)
		{
			double px, py;

			cColorb color = newTexture->m_image.getPixelColor(u, v);
			double height = -1;
			// The height should only be set if the pixel matches
			if (color.getB() == blue && color.getR() == red && color.getG() == green) 
			{
					height = 0.1;
			}
			else if (blue == 255 && red == 0 && green == 0) // Or if it's the ground
			{
					height = 0;
			}
			// compute the position of the vertex
			px = size * (double) u - offsetU;
			py = size * (double) v - offsetV;

			// create new vertex		
			unsigned int index = obj->newVertex(px, py, height);
			cVertex* vertex = obj->getVertex(index);
			
			vertex->setTexCoord(double(u) / double(texSizeU), double(v) / double(texSizeV));
		}
	}

	// Create a triangle based map using the above pixels
	for (int v = 0; v < (texSizeV - 1); v++)
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

	// This is the "size" of the object
	cVector3d span = cSub(max, min);
	size = cMax(span.x, cMax(span.y, span.z));

	// We'll center all vertices, then multiply by this amount,
	// to scale to the desired size.
	double scaleFactor = MESH_SCALE_SIZE / size;
	obj->scale(scaleFactor);

	// compute size of object again
	obj->computeBoundaryBox(true);

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
	// Check bounds
	if (cameraDistance < 0.1)
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

	// compute lookat position
	cVector3d lookat = cameraPosition;

	// define role orientation of camera
	cVector3d up(0.0, 0.0, 1.0);

	// set new position to camera
	camera->set(pos, lookat, up);

	// recompute global positions
	world->computeGlobalPositions(true);

	if (tool != NULL) // update tool position
		tool->setPos(-cameraDistance, 0.0, 0.0);
}