

/*
=====================================================================

Book				: Learning Physics Modeling with PhysX (ISBN: 978-1-84969-814-6)
Author				: Krishna Kumar
Compiler used		: Visual C++ 2010 Express
PhysX SDK version	: 3.3.0 
Source code name	: ch10_1_PVD
Reference Chapter	: Chapter-10: PhysX Visual Debugger

Description			: This example demonstrates how to connect your PhysX application with 'PhysX Visual Debugger'.
					  PVD is used for debugging and visualization of PhysX simulation. It can be used for run-time
					  visualization of PhysX scene as well as its data can be saved as file for later on analysis.     		
					  	
					  	
					  Navigation Controls:-
					  Mouse Left-Button Drag  : Scene orbital navigation
					  Mouse Right-Button Drag : Scene zoom-in/zoom-out
				
=====================================================================
*/




#include <iostream> 
#include <PxPhysicsAPI.h> //Single header file to include all features of PhysX API 
#include <GL/freeglut.h>  //OpenGL window tool kit 
#include "RenderBuffer.h" //Used for rendering PhysX objetcs 


//-------Loading PhysX libraries (32bit only)----------//

#ifdef _DEBUG //If in 'Debug' load libraries for debug mode 
#pragma comment(lib, "PhysX3DEBUG_x86.lib")				//Always be needed  
#pragma comment(lib, "PhysX3CommonDEBUG_x86.lib")		//Always be needed
#pragma comment(lib, "PhysX3ExtensionsDEBUG.lib")		//PhysX extended library 
#pragma comment(lib, "PhysXVisualDebuggerSDKDEBUG.lib") //For PVD only 

#else //Else load libraries for 'Release' mode
#pragma comment(lib, "PhysX3_x86.lib")	
#pragma comment(lib, "PhysX3Common_x86.lib") 
#pragma comment(lib, "PhysX3Extensions.lib")
#pragma comment(lib, "PhysXVisualDebuggerSDK.lib")
#endif

using namespace std;
using namespace physx; 


//========== Global variables ============//

int gWindowWidth  = 800; //Screen width
int gWindowHeight = 600; //Screen height


//---Scene navigation----
int gOldMouseX = 0;
int gOldMouseY = 0;

bool isMouseLeftBtnDown  = false;
bool isMouseRightBtnDown = false;

float gCamRoateX	= 15; 
float gCamRoateY	= 0;
float gCamDistance	= -50;
//------------------------

int oldTimeSinceStart = 0;
float mAccumulator = 0.0f;


static PxPhysics*				gPhysicsSDK = NULL;			//Instance of PhysX SDK
static PxFoundation*			gFoundation = NULL;			//Instance of singleton foundation SDK class
static PxDefaultErrorCallback	gDefaultErrorCallback;		//Instance of default implementation of the error callback
static PxDefaultAllocator		gDefaultAllocatorCallback;	//Instance of default implementation of the allocator interface required by the SDK
PxScene*						gScene = NULL;				//Instance of PhysX Scene				
PxReal							gTimeStep = 1.0f/60.0f;		//Time-step value for PhysX simulation 



//========== PhysX function prototypes ===========//

void InitPhysX();		//Initialize the PhysX SDK and create actors. 
void StepPhysX();		//Step PhysX simulation
void ShutdownPhysX();	//Shutdown PhysX SDK



//Functions for glut callbacks
void OnRender();					//Display callback for the current glut window
void OnIdle();						//Called whenever the application is idle
void OnReshape(int, int);			//Called whenever the application window is resized
void OnShutdown();					//Called on application exit
void OnMouseMotion(int,int);		//Called when the mouse is moving
void OnMousePress(int,int,int,int); //Called when any mouse button is pressed

void ConnectPvd();		//Use this function for real-time visualization of PhysX simulation (Optional and 'Debug' mode only) 
void SavePvdStream();	//Use this function for saving PVD stream as file



//Creates stack of boxes.
void CreateBoxStack(PxVec3 posOffset, PxU32 size, PxReal halfExt)
{
	PxMaterial  *material = gPhysicsSDK->createMaterial(0.5f,0.5f,0.5f);
 	
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExt);
			PxRigidDynamic* body = PxCreateDynamic(*gPhysicsSDK, PxTransform(localTm.p+posOffset),
			PxBoxGeometry(halfExt,halfExt,halfExt), *material, 1.0f);
		
			gScene->addActor(*body);
		}
	}
	
}


void main(int argc, char** argv) 
{

	glutInit(&argc, argv);								//Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);		//Enable double buffering
	glutInitWindowSize(gWindowWidth, gWindowHeight);	//Set window's initial width & height
	
	glutCreateWindow("Learning Physics Modeling with PhysX (ISBN: 978-1-84969-814-6) Examples"); // Create a window with the given title

	InitPhysX();

	glutDisplayFunc(OnRender);	//Display callback for the current glut window
	glutIdleFunc(OnIdle);		//Called whenever the application is idle
	glutReshapeFunc(OnReshape); //Called whenever the app window is resized

	glutMouseFunc(OnMousePress);	//Called on mouse button event
	glutMotionFunc(OnMouseMotion);	//Called on mouse motion event 

	glutMainLoop();				//Enter the event-processing loop
	atexit(OnShutdown);			//Called on application exit 
}




void InitPhysX() 
{
	//Creating foundation for PhysX
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
	
	//Creating instance of PhysX SDK
	gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale() );

	if(gPhysicsSDK == NULL) 
	{
		cerr<<"Error creating PhysX3 device, Exiting..."<<endl;
		exit(1);
	}



	//Creating scene
	PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());		//Descriptor class for scenes 

	sceneDesc.gravity		= PxVec3(0.0f, -9.8f, 0.0f);			//Setting gravity
	sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);		//Creating default CPU dispatcher for the scene
	sceneDesc.filterShader  = PxDefaultSimulationFilterShader;		//Creating default collision filter shader for the scene
	
	gScene = gPhysicsSDK->createScene(sceneDesc);					//Creating a scene 

	ConnectPvd();
	
	//This will enable basic visualization of PhysX objects like- actors collision shapes and their axes. 
	//The function PxScene::getRenderBuffer() is used to render any active visualization for scene.
	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE,				1.0);	//Global visualization scale which gets multiplied with the individual scales
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES,	1.0f);	//Enable visualization of actor's shape
	gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES,		1.0f);	//Enable visualization of actor's axis

	
	//Creating PhysX material (staticFriction, dynamicFriction, restitution)
	PxMaterial* material = gPhysicsSDK->createMaterial(0.5f,0.5f,0.5f);


	//1)-create static plane	 
	PxTransform planePos =	PxTransform(PxVec3(0, 0, 0),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
	PxRigidStatic* plane =  gPhysicsSDK->createRigidStatic(planePos);	
	
	plane->createShape(PxPlaneGeometry(), *material);
	gScene->addActor(*plane);

	//Create stack of boxes
	CreateBoxStack(PxVec3(0),10,1);

		
}


void StepPhysX()					//Stepping PhysX
{ 
	gScene->simulate(gTimeStep);	//Advances the simulation by 'gTimeStep' time
	gScene->fetchResults(true);		//Block until the simulation run is completed
} 


void ShutdownPhysX()				//Shutdown PhysX
{
	gPhysicsSDK->release();			//Removes any actors,  particle systems, and constraint shaders from this scene
	gFoundation->release();			//Destroys the instance of foundation SDK
}




void OnRender() 
{
	 int timeSinceStart = glutGet(GLUT_ELAPSED_TIME); 
     float deltaTime = (timeSinceStart - oldTimeSinceStart)/1000.0f;
     oldTimeSinceStart = timeSinceStart;
	
	 mAccumulator  += deltaTime;


	while(mAccumulator > gTimeStep) 
	{
		mAccumulator -= gTimeStep;
		StepPhysX(); 
	}
	
	//Update PhysX	
	if(gScene) 
		StepPhysX(); 

	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	
	glTranslatef(0,0,gCamDistance);
	glRotatef(gCamRoateX,1,0,0);
	glRotatef(gCamRoateY,0,1,0);
	

	RenderData(gScene->getRenderBuffer());
	
	glutSwapBuffers();
}



void ConnectPvd()	//Function for the visualization of PhysX simulation (Optional and 'Debug' mode only) 
{
	// check if PvdConnection manager is available on this platform
	if(gPhysicsSDK->getPvdConnectionManager() == NULL)
		return;

	// setup connection parameters
	const char*     pvd_host_ip = "127.0.0.1";  // IP of the PC which is running PVD
	int             port        = 5425;         // TCP port to connect to, where PVD is listening
	unsigned int    timeout     = 100;          // timeout in milliseconds to wait for PVD to respond,
												// consoles and remote PCs need a higher timeout.
	PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();

	// and now try to connect
	debugger::comm::PvdConnection* theConnection = PxVisualDebuggerExt::createConnection(gPhysicsSDK->getPvdConnectionManager(),
	pvd_host_ip, port, timeout, connectionFlags);

	if (theConnection)
         cout<<"PVD Connection over TCP/IP Successful!\n";
	else
		cout<<"PVD Connection over TCP/IP failed!\nPlz start PVD software first, then launch your PhysX application.";

}

void SavePvdStream()
{
	// check if PvdConnection manager is available on this platform
	if(gPhysicsSDK->getPvdConnectionManager() == NULL)
		return;
	// setup connection parameters
	const char*     filename = "D:\\PvdCapture.pxd2";  // filename where the stream will be written to

	PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();

	// and now try to connect
	debugger::comm::PvdConnection* theConnection = PxVisualDebuggerExt::createConnection(gPhysicsSDK->getPvdConnectionManager(),
    filename, connectionFlags);

	if (theConnection)
         cout<<"File Connection Successful!\n";
	else
		cout<<"File Connection failed!\n";
}


void OnReshape(int w, int h) 
{
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)w / (GLfloat)h, 0.1f, 100000.0f);
	glMatrixMode(GL_MODELVIEW);
}


void OnIdle() 
{
	glutPostRedisplay();
}

void OnShutdown() 
{
	ShutdownPhysX();
}



void OnMouseMotion(int curMouseX, int curMouseY) 
{
	if(isMouseLeftBtnDown)
	{
		 gCamRoateY += (curMouseX - gOldMouseX)/5.0f;
		 gCamRoateX += (curMouseY - gOldMouseY)/5.0f;
	}

	if(isMouseRightBtnDown)
	{
		gCamDistance -= (curMouseY - gOldMouseY)/5.0f;
	}

	gOldMouseX = curMouseX;
	gOldMouseY = curMouseY;

}


void OnMousePress(int mouseBtn, int mouseBtnState, int curMouseX, int curMouseY)
{
	if (mouseBtnState == GLUT_DOWN) 
	{
		if(mouseBtn== GLUT_LEFT_BUTTON) 
			isMouseLeftBtnDown = true;
		
		else if(mouseBtn == GLUT_RIGHT_BUTTON)
			isMouseRightBtnDown = true;
		
		gOldMouseX = curMouseX;
		gOldMouseY = curMouseY;

	}

	if (mouseBtnState == GLUT_UP ) 
	{
	  	isMouseLeftBtnDown = false;
		isMouseRightBtnDown = false;
	}

	//cout<<mouseBtn<<" "<<mouseBtnState<<" "<<x<<"|"<<y<<"\n";
}
