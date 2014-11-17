#include "tetgen.h"
#include "scene/scene.h"
#include "render/light.h"
#include "render/Camera.h"
#include "render/OpenGLRenderInterface.h"
#include "scene/DecoSoftBody.h"
#include "scene/DecoSoftBodyCorotationLinearFEM.h"
#include "scene/ActiveSoftBody.h"
#include "render/RenderMiscs.h"
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <istream>
#include "controller/MuscleLengthPlanner.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "utility/utility.h"
#include "scene/MovieRecorder.h"
using namespace google;


const int WIN_WIDTH = 800;
const int WIN_HEIGHT = 600;
const double FRAMES_PER_SECOND = 100;
const double gTimeStep = 1.0 / FRAMES_PER_SECOND;

bool gIsUpdatePaused = true;
int gFrameCounter = 0;
DecoScene gScene;
SoftObjRenderOption gRenderOption = RO_Mesh;

DecoRenderInterface* gRI = NULL;
DecoCamera* gCamera = NULL;

bool gIsLeftButtonDown = false;
bool gIsRightButtonDown = false;
bool gIsMiddleButtonDown = false;
bool gIsMouseMotion = false;

int gCurrentX, gCurrentY;
int gStartX, gStartY;
double gCurrentStack = PI / 2, gCurrentSlice = PI / 2;
double gSavedStack, gSavedSlice;
float gCameraRadius = 30;
VectorXd gMuscleLength;

ActiveSoftBody* gObj = NULL;
ActiveSoftBody* gTargetObj = NULL;

vector<vector<double> > gDesiredMuscleLength;
vector<vector<double> > gCurrentMuscleLength;
vector<vector<double> > gTargetError;

MovieRecorder gRecorder;
bool gIsRecording = false;

enum SelectMode
{
	SM_None,
	SM_Point,
	SM_Edge,
	SM_Triangle,
	SM_Object,
	SM_Max
};

#define SelectNone 0
#define SelectPoint 1
#define SelectEdge 2
#define SelectTriangle 3
#define SelectObject 4

SelectMode gSelectMode = SM_None;

class PointTarget
{
public:
	PointTarget() : mIsMoving(false), mTime(0)
	{

	}
	PointTarget(ActiveSoftBody* obj) : mObj(obj)
	{

	}
	void SetControlObject(ActiveSoftBody* obj)
	{
		mObj = obj;
	}
	int GetNumTargets() const
	{
		return static_cast<int>(mTargetPosition.size());
	}
	void Render(DecoRenderInterface* RI)
	{
		int numTargets = static_cast<int>(mVertexIndex.size());
		if (!numTargets)
			return;
		vector<vector3> allVertices;
		vector<vector3> targetVertices;
		for (int i = 0; i < numTargets; ++i)
		{
			DecoSoftObjectNode node = mObj->GetNode(mVertexIndex[i]);
			allVertices.push_back(node.mPos);
			targetVertices.push_back(vector3(mTargetPosition[i][0], mTargetPosition[i][1], mTargetPosition[i][2]));
		}
		DecoVertexBuffer vb;
		vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
		RI->SetColor(DecoColor(0xff00ff00));
		RI->SetLineWidth(5);
		RI->DrawPrimitive(PT_PointList, allVertices.size(), DT_SolidPolygon, &vb);

		vb.SetVertexInformation(targetVertices.size(), &(targetVertices[0]));
		RI->SetColor(DecoColor(0xff0000ff));
		RI->DrawPrimitive(PT_PointList, targetVertices.size(), DT_SolidPolygon, &vb);

	}
	void AddVertexTarget(int id, const Vector3d& pos)
	{
		mVertexIndex.push_back(id);
		mTargetPosition.push_back(pos);
	}
	const vector<int>& GetVertexIndices() const
	{
		return mVertexIndex;
	}
	const vector<Vector3d>& GetTargetPositions() const
	{
		return mTargetPosition;
	}
	void Update(double timeStep)
	{
		if (mIsMoving)
		{
			Vector3d newTargetPosition;
			newTargetPosition[0] = mTargetPosition[0][0];
			newTargetPosition[1] = 5 * cos(10 * mTime);
			newTargetPosition[2] = 5 * sin(10 * mTime);
			mTargetPosition[0] = newTargetPosition; 

			mTime += timeStep;
		}
	}
	void ConstructTest1()
	{
		AddVertexTarget(146, Vector3d(-8, 10, -2));
		AddVertexTarget(9, Vector3d(-1, 3, 5));
	}
	void ConstructTest2()
	{
		AddVertexTarget(146, Vector3d(-8, 10, -2));
	}
	void ConstructMovingTest2()
	{
		Vector3d pt1(8, 0, -2);
		Vector3d pt2(8, 0, 2);
		Vector3d center = (pt1 + pt2) / 2.0;
		Vector3d v1 = pt1 - center;
		Vector3d v2 = pt2 - center;
		double theta = 0;
		double cosine = cos(theta);
		double sine = sin(theta);
		Matrix3d rot;
		rot << 1, 0, 0, 0, cosine, -sine, 0, sine, cosine;
		Vector3d targetP1 = center + rot * v1;
		Vector3d targetP2 = center + rot * v2;
		AddVertexTarget(146, targetP1);
		AddVertexTarget(62, targetP2);
		mIsMoving = true;
	}
	void ConstructMovingTest1()
	{
		AddVertexTarget(146, Vector3d(5, 1, -2));
		mIsMoving = true;
	}
	
private:
	ActiveSoftBody* mObj;
	vector<int> mVertexIndex;
	vector<Vector3d> mTargetPosition;
	bool mIsMoving;
	double mTime;
};


void DumpMuscleLengthForGraph()
{
	ofstream outMatlabFile("muscleLength.m");
	char color[] = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
	
	int numDofs = static_cast<int>(gDesiredMuscleLength.size());
	int numFrames = static_cast<int>(gDesiredMuscleLength[0].size());
	
	outMatlabFile << "t = [";
	for (int i = 0; i < numFrames - 1; ++i)
	{
		outMatlabFile << i << ", ";
	}
	outMatlabFile << numFrames - 1 << "];" << std::endl;
	int numMuscles = gObj->GetNumMuscleFibers();
	for (int i = 0; i < numMuscles; ++i)
	{
		const MuscleFiber& fiber = gObj->GetMuscleFiber(i);
		int ithColor = 0;
		for (int j = 0; j < numDofs; ++j)
		{
			const MuscleDof& dof = gObj->GetMuscleDof(j);

		}
		outMatlabFile << "title('" << i << "th fiber');" << std::endl; 
		outMatlabFile << "figure;" << std::endl;
	}
	int numTargets = static_cast<int>(gTargetError.size());
	int ithColor = 0;
	for (int i = 0; i < numTargets; ++i)
	{
		outMatlabFile << "error = ";
		outMatlabFile << gTargetError[i]; 
		outMatlabFile << "plot(t, error, '" << color[ithColor] << "');" << std::endl;
		outMatlabFile << "hold on" << std::endl;
		ithColor = (ithColor + 1) % 7;;

	}
	outMatlabFile << "title('Target Errors');" << std::endl; 
	LOG(INFO) << "Finish dumping the matlat graph.";
};


void ReadScene(char* fileName)
{

}

void Simulate()
{
	MuscleLengthPlanner planner;
	VectorXd errorBeforePlanning;

	gMuscleLength = VectorXd::Constant(2, 0.6);
	gObj->SetDesiredMuscleLengthRelativeToRest(gMuscleLength);
	gScene.Update(gTimeStep);

	VectorXd desiredMuscleLength;
	VectorXd currentMuscleLength;
	
	gObj->GetDesiredMuscleLengthRelativeToRest(desiredMuscleLength);
	gObj->GetCurrentMuscleLengthRelativeToRest(currentMuscleLength);
	for (int i = 0; i < desiredMuscleLength.size(); ++i)
	{
		gDesiredMuscleLength[i].push_back(desiredMuscleLength[i]);
		gCurrentMuscleLength[i].push_back(currentMuscleLength[i]);
	}
	for (int i = 0; i < errorBeforePlanning.size(); ++i)
	{
		gTargetError[i].push_back(errorBeforePlanning[i]);
	}
}

void DumpResult()
{

}

void MenuCallBack(int id)
{
	switch(id)
	{
	case SelectNone:
		gSelectMode = SM_None;
		break;
	case SelectPoint:
		gSelectMode = SM_Point;
		break;
	case SelectEdge:
		gSelectMode = SM_Edge;
		break;
	case SelectTriangle:
		gSelectMode = SM_Triangle;
		break;
	case SelectObject:
		gSelectMode = SM_Object;
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void TimerFunc(int val)
{
	if (!gIsUpdatePaused)
	{
		LOG(INFO) << "-------------Simulating " << gFrameCounter << " frame--------------";
		Simulate();
		if (gIsRecording)
			gRecorder.Record();
		gFrameCounter++;
		glutPostRedisplay();
	}
	glutTimerFunc(static_cast<unsigned int>(1000 / FRAMES_PER_SECOND), TimerFunc, 0);		

}

void SelectFromScene(int x, int y, bool isCtrlPressed)
{
	switch (gSelectMode)
	{
	case SM_Point:
		gScene.SelectPoints(x, y, isCtrlPressed);
		break;
	default:
		break;
	}
	
}


void init(void) 
{

	gRI = gScene.RI;
	gRI->Initialize();
	DecoRenderMisc::Initialize(gRI);
	gCamera = gScene.camera;
	gCamera->set(vector3(0, 0, gCameraRadius), vector3(0, 0, 0), vector3(0, 1, 0));
	gCamera->setFrustum(60, WIN_WIDTH / static_cast<float>(WIN_HEIGHT), 1.f, 1000.f);
	gCamera->save();


	gObj = new ActiveSoftBody();
	gObj->ConstructTestBody(gTimeStep, "../../../../models/bar.1");
	gObj->Scale(vector3(1.0, 1.5, 1.0));
	gObj->ConstructTestConstraint();
	gScene.AddSceneObject(gObj);


	gCurrentMuscleLength.resize(gObj->GetNumMuscleDofs());
	gDesiredMuscleLength.resize(gObj->GetNumMuscleDofs());
	gRecorder.SetScene(&gScene);
	glutTimerFunc(static_cast<unsigned int>(1000 / FRAMES_PER_SECOND), TimerFunc, 0);	
}
void display(void)
{
	gRI->Clear();	
	gScene.SetSoftObjectRenderOption(gRenderOption);
	gScene.Render();

	const int pathLen = 512;
	char fileName[pathLen];
	memset(fileName, 0, pathLen * sizeof(char));
	sprintf_s(fileName, "../../../tmpPic\\FEM1%04d.jpg", gFrameCounter);
	if (gFrameCounter % static_cast<int>(FRAMES_PER_SECOND / 30) == 0)
		gRI->SaveToImage(fileName);

	glutSwapBuffers();
}

void reshape(int w, int h)
{
	gRI->SetViewport(0, 0, w, h);
}

void keyboard(unsigned char key, int x, int y)
{
	float step = 2.0;
	vector<DecoSceneObject*> allObjs;
	switch(key)
	{
	case '1':
		gRenderOption = RO_Points;
		break;
	case '2':
		gRenderOption = RO_Edges;
		break;
	case '3':
		gRenderOption = RO_Mesh;
		break;
	case '4':
		gRenderOption = RO_Tetrahedra;
		break;
	case '5':
		gRenderOption = RO_ID;
		break;
	case '6':
		gRenderOption = RO_Barycentric;
		break;
	case '7':
		gScene.ToggleForceRenderOption(FRRT_Contact);
		break;
	case '8':
		gScene.ToggleForceRenderOption(FRRT_Gravity);
		break;
	case '9':
		gScene.ToggleForceRenderOption(FRRT_Elastic);
		break;
	case '0':
		gScene.ToggleForceRenderOption(FRRT_Muscle);
		break;
	case '-':
		gScene.ToggleForceRenderOption(FRRT_Total);
		break;
	case ' ':
		gIsUpdatePaused = !gIsUpdatePaused;
	case 'c':
		gCamera->save();
		gSavedSlice = gCurrentSlice;
		gSavedStack = gCurrentStack; 
		break;
	case 'b':
		gCamera->load();
		gCurrentSlice = gSavedSlice;
		gCurrentStack = gSavedStack;  
		break;
	case 'r':
		gCamera->set(vector3(0, 0, 30), vector3(0, 0, 0), vector3(0, 1, 0));
		gCurrentSlice = gCurrentStack = PI / 2;
		gCameraRadius = 30;
		break;
	case 's':
		gScene.SaveDeformedObjectsShape();
		break;
	case 'w':
		gScene.SaveDeformedObjects("snapshot");
		LOG(INFO) << "Finish writing files";
		break;
	case 'm':
		DumpMuscleLengthForGraph();
		break;
	case 'l':
		gScene.EraseAllSceneObject();
		gScene.LoadDeformedObjects("snapshot");
		gScene.GetAllSceneObjects(allObjs);
		gObj = static_cast<ActiveSoftBody*>(allObjs[0]);
		LOG(INFO) << "Finish reading files";
		break;
	case 'R':
		gRecorder.Initialize();
		LOG(INFO) << "start/pause recording...";
		gIsRecording = !gIsRecording;
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) 
{
	int mod = glutGetModifiers();

	switch (button) 
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			gIsLeftButtonDown = true;
			gCurrentX = gStartX = x;
			gCurrentY = gStartY = WIN_HEIGHT - y;
		}
		else if (state == GLUT_UP)
		{
			gIsMouseMotion = false;
			gIsLeftButtonDown = false;
			if (abs(x - gCurrentX) < 5 && abs(WIN_HEIGHT - y - gCurrentY) < 5)
			{
				bool isCtrlPressed = (mod == GLUT_ACTIVE_CTRL);
				SelectFromScene(x, WIN_HEIGHT - y, isCtrlPressed);
			}
		}

		break;

	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN)
		{
			gIsRightButtonDown = true;

			gCurrentX = gStartX = x;
			gCurrentY = gStartY = WIN_HEIGHT - y;
		}
		else if (state == GLUT_UP)
		{
			gIsMouseMotion = false;
			gIsRightButtonDown = false;

		}
		break;
	case GLUT_MIDDLE_BUTTON:
		if (state == GLUT_DOWN)
		{
			gIsMiddleButtonDown = true;
			gCurrentX = gStartX = x;
			gCurrentY = gStartY = WIN_HEIGHT - y;
		}
		else if (state == GLUT_UP)
		{
			gIsMouseMotion = false;
			gIsMiddleButtonDown = false;
		}

		break;
	default:
		break;
	}
	glutPostRedisplay();

}

void motion(int x, int y)
{
	gIsMouseMotion = true;
	int mod = glutGetModifiers();
	float scale = 20.f;
	if (gIsLeftButtonDown)
	{
		if (mod == GLUT_ACTIVE_ALT)
		{
			float stackrad = (WIN_HEIGHT - y - gCurrentY) / scale;
			float slicerad = (x - gCurrentX) / scale;

			vector3 eye = gCamera->getEye();
			vector3 dir = -gCamera->getLookAtDir();
			vector3 up = gCamera->getUpDir();
			float r = gCameraRadius;
			vector3 lookAtPt = eye + r * dir;

			vector3 newEye = lookAtPt + vector3(r * sin(gCurrentStack) * cos(gCurrentSlice), r * cos(gCurrentStack), r * sin(gCurrentStack) * sin(gCurrentSlice));

			gCamera->set(newEye, lookAtPt, up);

			gCurrentStack += stackrad;
			gCurrentSlice += slicerad;
		}
	}
	else if (gIsMiddleButtonDown)
	{
		if (mod == GLUT_ACTIVE_ALT)
		{
			gCamera->slide(-(x - gCurrentX) / scale, -(WIN_HEIGHT - y - gCurrentY) / scale, 0, TRUE);
		}

	}
	else if (gIsRightButtonDown)
	{
		if (mod == GLUT_ACTIVE_ALT)
		{
			gCameraRadius -= (WIN_HEIGHT - y - gCurrentY) / scale;
			gCamera->slide(0, 0, -(WIN_HEIGHT - y - gCurrentY) / scale, TRUE);
		}
	}
	gCurrentX = x;
	gCurrentY = WIN_HEIGHT - y;


	glutPostRedisplay();
}

void wheel(int wheelnum, int dir, int x, int y)
{
	gCamera->slide(0, 0, -2 * dir, TRUE);
	glutPostRedisplay();
}


int main(int argc, char** argv)
{
	ParseCommandLineFlags(&argc, &argv, true);
	InitGoogleLogging(argv[0]);
	
	FLAGS_alsologtostderr = true;
	FLAGS_log_dir = "../../../logs";
	FLAGS_v = 2;
	glutInit(&argc, argv);

	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_ALPHA | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowSize (WIN_WIDTH, WIN_HEIGHT); 
	glutInitWindowPosition (100, 100);
	glutCreateWindow (argv[0]);
	if (glewInit() != GLEW_OK)
	{
		printf("glewInit failed. Exiting...\n");
		exit(1);
	}

	if (!glewIsSupported( "GL_VERSION_2_0 "
		"GL_ARB_texture_rectangle "
		"GL_ARB_texture_float "
		"GL_NV_float_buffer "
		"GL_NV_depth_buffer_float "))
	{
		printf("Unable to load the necessary extensions\n");
		printf("This sample requires:\n");
		printf("OpenGL version 2.0\n");
		printf("GL_ARB_texture_rectangle\n");
		printf("GL_ARB_texture_float\n");
		printf("GL_NV_float_buffer\n");
		printf("GL_NV_depth_buffer_float\n");
		printf("Exiting...\n");
		exit(1);
	}

	init ();
	glutDisplayFunc(display); 
	glutReshapeFunc(reshape); 
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutMouseWheelFunc(wheel);
	glutCreateMenu(MenuCallBack);
	glutAddMenuEntry("Select None", SelectNone);
	glutAddMenuEntry("Select Point", SelectPoint);

	glutAttachMenu(GLUT_RIGHT_BUTTON);
	glutMainLoop();
	return 0;

}
