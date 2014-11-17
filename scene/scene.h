#ifndef SCENE_H
#define SCENE_H

#include "stdafx.h"
#include "SceneObj.h"
#include "grid.h"
#include "DecoSoftBody.h"
#include "render/RenderInterface.h"
#include "render/Camera.h"
#include <vector>
#include "collision/CollisionDetector.h"
#include "collision/LCPFormulator.h"
#include "ForceRecorderRenderType.h"

using namespace std;

class DecoSceneNode;
class DecoScene;
class DecoArticulatedObject;
class LDIPixelTranslated;
class CollisionHandler;
class LCPFormulator;
class ContactGroup;
class ActiveSoftBody;

typedef unsigned long DWORD;


class DecoZone
{
protected:
	BYTE zoneID;
	BOOL bIsVisible;
public:
	DecoZone();
	virtual ~DecoZone();
	void Render(DecoRenderInterface* RI, DecoCamera* camera);
};



class DecoSceneNode
{
private:
	BOOL bIsLeaf;
	DecoSceneNode* Parent;
	DecoSceneNode* leftChild;
	DecoSceneNode* rightChild;
	vector<DecoSceneObject*> sceneObjs;
	vector<DecoLight*> sceneLights;
	DecoScene* scene;
	void RenderAllSceneObjects(DecoRenderInterface* RI, DecoCamera* camera, SceneObjectType objType = AllT, DecoDrawType drawType = DT_SolidPolygon);
public:
	DecoSceneNode(DecoScene* scn, BOOL isLeaf, DecoSceneNode* lchild = NULL, DecoSceneNode* rchild = NULL);
	~DecoSceneNode();
	void Render(DecoRenderInterface* RI, DecoCamera* camera, SceneObjectType objType = AllT, DecoDrawType drawType = DT_SolidPolygon);
	void RenderAllSceneObjectsPlain(DecoRenderInterface* RI, DecoCamera* camera, SceneObjectType objType = AllT);
	void GetRelevantLights(const DecoRenderData* obj, INT& numLights, DecoLight** lights);
	DecoSceneObject* GetFirstSceneObj()
	{
		if (sceneObjs.size())
			return sceneObjs[0];
		else
			return NULL;
	}
	vector<DecoSceneObject*>& GetSceneObjects()
	{
		return sceneObjs;
	};
	friend DecoArchive& operator<< (DecoArchive& Ar, const DecoSceneNode& node);
	friend DecoArchive& operator>> (DecoArchive& Ar, DecoSceneNode& node);
};

class DecoSolidObject;
class GridField;
class TetGridField;

class DecoScene
{
private:
	DecoSceneNode* rootSceneNode;
	DOUBLE timeStep;
	vector<DecoSceneObject*> selectedObjs;

	Box scnBox;
	DOUBLE time;

	int mLDIWidth;
	int mLDIHeight;
    bool mIsCollisionDisabled;

	vector3 PointScreen2Camera(const vector2& pt) const;

	void updateContactPolygon();
	void updateCenterOfPressure();

public:
	DecoRenderInterface* RI;
	DecoCamera* camera;
	CollisionHandler* mCollisionHandler;
    Grid mGrid;
#ifdef _LCP_CONTACT
    LCPFormulator* mLCPFormulator;
#endif
	//static DecoScene* GetSingleton();
	//static void DestroySingleton();
	DecoScene();
	~DecoScene();
	void Render(BOOL bViewModel = TRUE, BOOL bRenderAuxilary = TRUE);
	void RenderLDIModels(int width, int height, AXIS lookAxis, SoftObjRenderOption opt);
	void Reshape(int w, int h)
	{
		assert(RI);
		RI->SetViewport (0, 0, (GLsizei) w, (GLsizei) h);
	}
    void MuteGravity();
    void SetSpatialSubsivision(int xGrids, int yGrids, int zGrids);
    void SetGrid(const Box& box);
    const Grid& GetSpatialGrid() const;

	// next two functions should be set when initialize in sequence
	void SetSceneBox(const Box& _scnBox)
	{
		scnBox = _scnBox;
	}

	//////////////////////////////////////////////////
	Box GetSceneBox() const
	{
		return scnBox;
	}
	DOUBLE GetTimeStep() const
	{
		return timeStep;
	}
	void SetTimeStep(double step);
	DecoSceneObject* GetFirstObject()const;

	void PointScreen2WorldRay(const vector2& pt, vector3& outOrigin, vector3& outDir) const;
	vector3 vectorScreen2World(const vector2& start, const vector2& end, DOUBLE startDistance = 0, Plane constraint[] = NULL, INT numConstraints = 0) const;
	DecoSceneObject* SelectObj(FLOAT x, FLOAT y) const; 
	void SelectPoints(FLOAT x, FLOAT y, bool bAppend);
    void SelectMuscles(FLOAT x, FLOAT y, bool bAppend);
	void DeselectAll ();
	BOOL IsSomethingSelected() const;
	// return FALSE if there is no intersection
	void Destroy(DecoSceneObject* obj);
	void DestroyAll();
	//erase from list but deletion is left to decoObjFactory
	BOOL EraseAllSceneObject();
	DecoSceneObject* GetFocusObject()const;
	void SetFocusObject(DecoSceneObject* obj);
	void AddSceneObject(DecoSceneObject* solid);
	void PrepareSolidJacobian();
	void DumpObjFiles(const std::string& fileName);
	void GatherAndOutputStatistics(int ithSample1, int numSample1, double p1, int ithSample2, int numSample2, double p2);
	vector3 GetLDIPixelPosition(const LDIPixelTranslated& px, AXIS axis);
	DOUBLE GetLDIPixelArea(AXIS axis) const;
	DOUBLE GetLDIPixelDepthScale(AXIS axis) const;
	void Update(DOUBLE seconds);
    void DisableCollisionDetection();
#ifdef _LCP_CONTACT
    ContactGroup* GetContactGroup(DecoSceneObject* obj) const;
	void GetContactForce(DecoSceneObject* obj, const VectorXd& normalMag, const VectorXd& tangentialMag, NodalForceT& normalForces, NodalForceT& frictionForces);
    void UpdateStage1(DOUBLE seconds);
    void UpdateStage2(DOUBLE seconds, VectorXd* normalForce = NULL, VectorXd* frictionForce = NULL, VectorXd* lambda = NULL, bool bValidate = false);
	void GetInitialGuessForLCPSolver(ActiveSoftBody* body, vector<int>& isEqualityOnZ);
#endif
	DOUBLE GetCurrentTime();
	void Reset();
	void ClearContactForce();
	void ResetCurrentTime();
	const std::vector<CollisionInfo>& GetCollisionInfos();
	DecoSceneObject* GetSceneObj(const char* name);
	void GetAllSceneObjects( vector<DecoSceneObject*> &allObjs);
	DecoSoftObject* GetSoftObject(int index);
	void SetSoftObjectRenderOption(SoftObjRenderOption opt);
    void MoveGrid(double x, double y, double z);
    void MoveGridTo(double x, double y, double z);
    vector3 GetGridCenter() const;
	void ToggleForceRenderOption(ForceRecorderRenderType type);
    void RecordMovie(DecoArchive& Ar, double playerVersion);
	void SaveDeformedObjects(const string& filename);
	void LoadDeformedObjects(const string& filename);
	void SaveDeformedObjectsShape();
	void Serialize(DecoArchive& Ar);
	void Deserialize(DecoArchive& Ar);

};

#endif
