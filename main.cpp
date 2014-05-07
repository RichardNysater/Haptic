/**
	This is the main file for a haptics course project
	and is free software; you can redistribute it (and the rest of the files) and/or modify
	it under the terms of the GNU General Public License("GPL") version 2
	as published by the Free Software Foundation.
	
	@author	Richard Nysater
	@version 1.0 2014-05-07
*/

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

// Variables
vector<Field> fields; // Vector with all the force fields
LoadMap loadMap;
long long curFPS = 0, curHz = 0;
double FPS = 0, hapticHz = 0, velocity = 0;
bool worldTransparent = false, roofTransparent = true;


int displayW = 0, displayH = 0;
bool programRunning = false, programFinished = false, flagCameraInMotion = false;
double cameraAngleH, cameraAngleV, cameraDistance, proxyRadius;
int mouseX, mouseY, mouseButton;
cLabel* FPSLabel, *HapticHzLabel; // FPS trackers
cBitmap* circuit;
cWorld* world;
cCamera* camera;
cMesh* walls, * roof, *switchWalls;
cLight *light;
cHapticDeviceHandler* handler; 
cGeneric3dofPointer* tool; 
cVector3d cameraPosition;

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

/**
	Initialize the tool and other haptic devices.
*/
const double initializeTools()
{
	handler = new cHapticDeviceHandler();
	cGenericHapticDevice* hapticDevice;
	handler->getDevice(hapticDevice, 0);

	cHapticDeviceInfo info;
	if (hapticDevice)
	{
		info = hapticDevice->getSpecifications();
	}
	tool = new cGeneric3dofPointer(world);
	camera->addChild(tool);

	//tool->setPos(-cameraDistance, 0.0, 0.0);
	
	tool->setHapticDevice(hapticDevice);
	
	tool->start();
	tool->setPos(-3.0, 0, 0.01); // Play around with this value
	tool->setWorkspaceRadius(1.0); // map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setRadius(0.03);
	tool->m_deviceSphere->setShowEnabled(false);
	proxyRadius = 0.03;
	tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);
	tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = true;
	
	tool->getProxy()->setProxyGlobalPosition(cVector3d(-0.878378, 0.621621, 0.05));
	const double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	const double stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;
	return stiffnessMax;
}

/**
	Initialize the camera and light.
*/
void initializeCamera()
{
	camera = new cCamera(world);
	world->addChild(camera);

	cameraAngleH = 0;
	cameraAngleV = 90;
	cameraDistance = 1.8 * MESH_SCALE_SIZE;
	updateCameraPosition();

	camera->setClippingPlanes(0.01, 10.0); // Min and max distance from camera

	light = new cLight(world);
	camera->addChild(light);
	light->setEnabled(true);
	light->setPos(cVector3d(0.0, 0.3, 0.3));
	light->setDir(cVector3d(-1.0, -0.1, -0.1));
	light->m_ambient.set(0.5, 0.5, 0.5);
	light->m_diffuse.set(0.8, 0.8, 0.8);
	light->m_specular.set(1.0, 1.0, 1.0);

	FPSLabel = new cLabel();
	FPSLabel->setPos(8, 15, 0);
	HapticHzLabel = new cLabel();
	HapticHzLabel->setPos(8, 30, 0);
	camera->m_front_2Dscene.addChild(FPSLabel);
	camera->m_front_2Dscene.addChild(HapticHzLabel);

}

/**
	Loads the world. This means the walls, invisible roof and forces.
*/
void loadWorld(const double stiffnessMax)
{
	walls = new cMesh(world);
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
	
	walls->setTransparencyLevel(0, true, true);
	switchWalls->setTransparencyLevel(100, true, true);
	walls->setStiffness(0.5 * stiffnessMax, true);
	roof->setStiffness(0.5 * stiffnessMax, true);
	switchWalls->setStiffness(0.5 * stiffnessMax, true);
}

/**
	Handles the actions of pressing buttons on the haptic device
*/
void checkButtons()
{
	bool userSwitch = tool->getUserSwitch(0);
	if (userSwitch) // Disable collision while the button is held
	{
		
		if (walls->getCollisionDetector() != NULL)
		{
			cerr << "Collision detection off" << endl;
			walls->setCollisionDetector(NULL);
		}

		if (switchWalls->getCollisionDetector() != NULL)
			switchWalls->setCollisionDetector(NULL);

		if (roof->getCollisionDetector() != NULL)
			roof->setCollisionDetector(NULL);
	}
	else
	{
		
		if (walls->getCollisionDetector() == NULL)
		{
			cerr << "Collision detection on" << endl;
			walls->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
		}
		if (switchWalls->getCollisionDetector() == NULL)
			switchWalls->createAABBCollisionDetector(1.01*proxyRadius, true, false);

		if (roof->getCollisionDetector() == NULL)
			roof->createAABBCollisionDetector(1.01*proxyRadius, true, false);
	}
}

/*
Updates the haptic feedback
*/
void updateHaptics(void)
{
	cPrecisionClock pclock;
	pclock.setTimeoutPeriodSeconds(1.0);
	pclock.start(true);

	cVector3d toolGlobalPos, toolLocalPos, prevToolGlobalPos, prevToolLocalPos;

	while (programRunning)
	{
		world->computeGlobalPositions(true);

		tool->updatePose();
		tool->computeInteractionForces();

		toolGlobalPos = tool->getDeviceGlobalPos();
		toolLocalPos = tool->getDeviceLocalPos();

		checkButtons();

		prevToolLocalPos = toolLocalPos;
		prevToolGlobalPos = toolGlobalPos;

		
		for (int ind = 0; ind < fields.size(); ++ind)
		{
			if (fields[ind].isInside(tool))
			{
				fields[ind].setVelocity(tool, velocity);
				//cerr << "Inside field: " << ind << " dir: " << fields[ind].direction() << endl;
				switch (fields[ind].direction())
				{
				case DOWN:
					//std::cerr << "Down with velocity " << velocity << std::endl;
					//tool->m_lastComputedGlobalForce.add(cVector3d(0, 0, -10*local_inversion));
					tool->m_lastComputedGlobalForce.x = velocity;
					break;
				case UP:
					//std::cerr << "Up with velocity " << velocity << std::endl;
					tool->m_lastComputedGlobalForce.x = -velocity;
					break;
				case LEFT:
					//std::cerr << "Left with velocity " << velocity << std::endl;
					tool->m_lastComputedGlobalForce.y = -velocity;
					break;
				case RIGHT:
					//std::cerr << "Right with velocity " << velocity << std::endl;
					tool->m_lastComputedGlobalForce.y = velocity;
					break;
				}
			}
		}
		
		tool->applyForces();
		curHz++;
		if (pclock.timeoutOccurred()) {
			FPS = curFPS;
			hapticHz = curHz;
			curHz = 0;
			curFPS = 0;
			pclock.start(true);
		}
	}
	programFinished = true;
}

/**
	This function is called when a key is pressed
*/
void keySelect(unsigned char key, int x, int y)
{
	if ((key == 27) || (key == 'x')) // Escape
	{
		close();
		exit(EXIT_SUCCESS);
	}
	else if (key == '1')
	{
		bool useTexture = !walls->getUseTexture();
		if (useTexture)
			cerr << "Wall textures activated" << endl;
		else
			cerr << "Wall textures deactivated" << endl;
		walls->setUseTexture(useTexture);
	}
	else if (key == '2')
	{
		bool useWireMode = !walls->getWireMode();
		if (useWireMode)
			cerr << "Wall wiremode activated" << endl;
		else
			cerr << "Wall wiremode deactivated" << endl;
		walls->setWireMode(useWireMode);
	}
	else if (key == '3')
	{
		bool switchWallsOn = !switchWalls->getShowEnabled();
		if (switchWallsOn)
			cerr << "Switch walls activated" << endl;
		else
			cerr << "Switch walls deactivated" << endl;
		switchWalls->setShowEnabled(switchWallsOn);
	}
	else if (key == '4')
	{
		if (walls->getCollisionDetector() == NULL)
		{
			cerr << "Collision detection activated" << endl;
			walls->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
		}
		else
		{
			cerr << "Collision detection deactived" << endl;
			walls->setCollisionDetector(NULL);
		}

		if (switchWalls->getCollisionDetector() == NULL)
			switchWalls->createAABBCollisionDetector(1.01*proxyRadius, true, false);
		else
			switchWalls->setCollisionDetector(NULL);

		if (roof->getCollisionDetector() == NULL)
			roof->createAABBCollisionDetector(1.01*proxyRadius, true, false);
		else
			roof->setCollisionDetector(NULL);
	}
	else if (key == '5')
	{
		if (worldTransparent)
		{
			cerr << "Walls are invisible" << endl;
			walls->setTransparencyLevel(0, true, true);
		}
		else
		{
			cerr << "Walls are visible" << endl;
			walls->setTransparencyLevel(100, true, true);
		}
		worldTransparent = !worldTransparent;
	}
	else if (key == '6')
	{
		if (roofTransparent)
		{
			cerr << "Roof is invisible" << endl;
			roof->setTransparencyLevel(0, true, true);
		}
		else
		{
			cerr << "Roof is visible" << endl;
			roof->setTransparencyLevel(100, true, true);
		}
		roofTransparent = !roofTransparent;
	}
}

/**
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

	cVector3d lookAt = cameraPosition;
	cVector3d up(0.0, 0.0, 1.0);
	camera->set(pos, lookAt, up);
	world->computeGlobalPositions(true);

	//if (tool != NULL) // update tool position
	//	tool->setPos(-cameraDistance, 0.0, 0.0);
}

/**
	Starts the opengl graphics
*/
void glut_start(int argc, char** argv)
{
	glutInit(&argc, argv);

	int screenW = glutGet(GLUT_SCREEN_WIDTH);
	int screenH = glutGet(GLUT_SCREEN_HEIGHT);
	int windowPosX = (screenW - WINDOW_SIZE_W) / 2; // Centered
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

	// Start simulation
	programRunning = true;

	cThread* hapticsThread = new cThread(); // create a thread which starts the main haptics rendering loop
	hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

	glutMainLoop(); 	// start the main graphics rendering loop
	close();
}

/**
	Updates the graphics displayed
*/
void updateGraphics(void)
{
	FPSLabel->m_string = "FPS: " + std::to_string((int) FPS); // Print the FPS (should be 30+)
	HapticHzLabel->m_string = "Haptic Hz: " + std::to_string((int) hapticHz); // Print the haptic update rate (needs to be 500+)

	walls->computeAllNormals(true);
	camera->renderView(displayW, displayH);

	glutSwapBuffers();

	if (programRunning)
	{
		glutPostRedisplay();
	}
	curFPS++;
}

/**
	This function is called when the window is resized by the user
*/
void resizeWindow(int w, int h)
{
	displayW = w;
	displayH = h;
	glViewport(0, 0, displayW, displayH);
}

/**
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

/**
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

/**
	Close the program
*/
void close(void)
{
	programRunning = false;
	while (!programFinished)
	{
		cSleepMs(100);
	}
	tool->stop();
}

/**
	The main function initializes everything.
*/
int main(int argc, char* argv [])
{
	initializeConstants();
	loadMap.initialize(argv);
	
	world = new cWorld(); // create a new world.
	world->setBackgroundColor(0.1, 0.1, 0.1);

	initializeCamera();

	loadMap.loadCircuitPicture(circuit, camera);

	const double stiffnessMax = initializeTools(); // Initialize the tool and get the max stiffness
	loadWorld(stiffnessMax); // Load the world (walls, roof, forces etc)

	glut_start(argc, argv); // Start rendering with OpenGL
	return (EXIT_SUCCESS);
}