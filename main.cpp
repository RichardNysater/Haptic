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
#include "loadMap.h"

using std::vector; using std::pair; using std::make_pair;
using std::cout; using std::endl; using std::cerr;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------
vector<Field> fields; // Vector with all the force fields
LoadMap loadMap;
long long iterations = 0;
bool worldTransparent = false, forcesTransparent = true;
int inversion = 1;// -1 if inverted

// Pre-existing variables
cLabel* rateLabel; // FPS tracker
double FPS = 0;
cWorld* world; // a world that contains all objects of the virtual environment
cCamera* camera; // a camera that renders the world in a window display
cMesh* walls, * roof, *switchWalls; // a mesh object used to create the height map
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

bool programFinished = false; // has exited haptics simulation thread



// DECLARED FUNCTIONS
void updateCameraPosition(); // update camera settings
void resizeWindow(int w, int h); // callback when the window display is resized
void keySelect(unsigned char key, int x, int y); // callback when a keyboard key is pressed
void mouseClick(int button, int state, int x, int y); // callback to handle mouse click
void mouseMove(int x, int y); // callback to handle mouse motion
void close(void); // function called before exiting the application
void updateGraphics(void); // main graphics callback
void updateHaptics(void); // main haptics loop
void glut_start(int argc, char** argv); // Start rendering
const double initializeTools(); // Initialize the tool and haptic devices
void loadWorld(const double); // Load the world

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
	proxyRadius = 0.03; 	// set the physical readius of the proxy.
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

	rateLabel = new cLabel();
	rateLabel->setPos(8, 24, 0);
	camera->m_front_2Dscene.addChild(rateLabel);

}

/*
Loads the world. This means the walls, invisible roof and forces.
*/
void loadWorld(const double stiffnessMax)
{
	walls = new cMesh(world); // create new meshes for the solid world and the forces
	roof = new cMesh(world);
	switchWalls = new cMesh(world);

	world->addChild(walls);	
	world->addChild(roof);
	world->addChild(switchWalls);

	if (loadMap.loadWalls(world, walls, proxyRadius, fields) != EXIT_SUCCESS)
	{
		cerr << "Error loading walls!" << endl;
		close();
	}
	if (loadMap.loadRoof(world, roof, proxyRadius) != EXIT_SUCCESS)
	{
		cerr << "Error loading roof!" << endl;
		close();
	}
	if (loadMap.loadSwitchWalls(world, switchWalls, proxyRadius) != EXIT_SUCCESS)
	{
		cerr << "Error loading switchwalls!" << endl;
		close();
	}
	
	walls->setTransparencyLevel(100, true, true); // walls should be visible
	switchWalls->setTransparencyLevel(100, true, true); // switchwalls should be visible
	walls->setStiffness(0.5 * stiffnessMax, true);
	roof->setStiffness(0.5 * stiffnessMax, true);
	switchWalls->setStiffness(0.5 * stiffnessMax, true);
}

/*
	Handles the actions of pressing buttons on the haptic device
*/
int checkButtons(int state, cVector3d toolLocalPos, cVector3d prevToolLocalPos)
{
	const int STATE_IDLE = 1;
	const int STATE_MOVE_CAMERA = 2;
	const int STATE_NOHAPTIC = 3;

	bool userSwitch = tool->getUserSwitch(0);
	bool hapticSwitch = tool->getUserSwitch(1);


	if (hapticSwitch) // Disable haptics while this button is held
	{
		walls->setHapticEnabled(false, true);
		roof->setHapticEnabled(false, true);
	}
	else
	{
		walls->setHapticEnabled(true, true);
		roof->setHapticEnabled(true, true);
	}

	if ((state == STATE_MOVE_CAMERA) && (!userSwitch)) // Stop moving camera
	{
		state = STATE_IDLE;
		walls->setHapticEnabled(true, true);
		roof->setHapticEnabled(true, true);
	}
	else if ((state == STATE_IDLE) && (userSwitch)) // Start moving camera
	{
		state = STATE_MOVE_CAMERA;
		walls->setHapticEnabled(false, true);
		roof->setHapticEnabled(false, true);
	}
	else if (state == STATE_MOVE_CAMERA) // Keep moving camera
	{
		cVector3d offset = toolLocalPos - prevToolLocalPos; // compute tool offset
		cameraDistance = cameraDistance - 2 * offset.x; // apply camera motion
		cameraAngleH = cameraAngleH - 40 * offset.y;
		cameraAngleV = cameraAngleV - 40 * offset.z;

		updateCameraPosition();
	}
	return state;
}


/*
Updates the haptic feedback
*/
void updateHaptics(void)
{
	cPrecisionClock pclock;
	pclock.setTimeoutPeriodSeconds(1.0);
	pclock.start(true);

	cPrecisionClock clock;
	clock.start(true);

	int counter = 0;

	int state = 1;

	cVector3d toolGlobalPos, toolLocalPos, prevToolGlobalPos, prevToolLocalPos; // Tool positions

	while (programRunning)
	{
		world->computeGlobalPositions(true); // compute global reference frames for each object

		tool->updatePose(); // update position, orientation of tool and compute forces
		tool->computeInteractionForces();

		toolGlobalPos = tool->getDeviceGlobalPos(); // update tool position
		toolLocalPos = tool->getDeviceLocalPos();

		state = checkButtons(state,toolLocalPos,prevToolLocalPos);

		prevToolLocalPos = toolLocalPos;
		prevToolGlobalPos = toolGlobalPos;

		//std::cerr << tool->getDeviceGlobalPos().x << " " << tool->getDeviceGlobalPos().y << " " << tool->getDeviceGlobalPos().z << std::endl;

		for (int ind = 0; ind < fields.size(); ++ind)
		{
			if (fields[ind].isInside(tool))
			{
				//cerr << "Inside field: " << ind << "dir: " << fields[ind].direction() << endl;
				switch (fields[ind].direction())
				{
				case DOWN:
					//std::cerr << "DOWN " << std::endl;
					//tool->m_lastComputedGlobalForce.add(cVector3d(0, 0, -10*local_inversion));
					tool->m_lastComputedGlobalForce.x = 3;
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
					tool->m_lastComputedGlobalForce.y = -3;
					break;
				case RIGHT:
					//std::cerr << "RIGHT " << std::endl;
					tool->m_lastComputedGlobalForce.x = 0;
					tool->m_lastComputedGlobalForce.y = 3;
					break;
				}
			}
		}

		tool->applyForces();


		if (pclock.timeoutOccurred()) {
			FPS = iterations;
			iterations = 0;
			pclock.start(true);
		}
	}
	programFinished = true;
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
		bool useTexture = walls->getUseTexture();
		walls->setUseTexture(!useTexture);
	}
	else if (key == '2')
	{
		bool useWireMode = walls->getWireMode();
		walls->setWireMode(!useWireMode);
	}
	else if (key == '3')
	{
		switchWalls->setShowEnabled(!switchWalls->getShowEnabled());
	}
	else if (key == '4')
	{
		inversion *= -1;
	}
	else if (key == '5')
	{
		if (worldTransparent)
		{
			walls->setTransparencyLevel(0, true, true);
		}
		else
		{
			walls->setTransparencyLevel(100, true, true);
		}
		worldTransparent = !worldTransparent;
	}
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
	char buffer[128];
	sprintf(buffer, "FPS: %.0lf ", FPS);
	rateLabel->m_string = buffer;
	// update walls normals
	walls->computeAllNormals(true);
	
	// render world
	camera->renderView(displayW, displayH);

	glutSwapBuffers();

	if (programRunning)
	{
		glutPostRedisplay();
	}
	iterations++;
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
The main function initializes everything.
*/
int main(int argc, char* argv [])
{
	initializeConstants();
	loadMap.initialize(argv);
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