#include "stdafx.h"
#include "Scene.h"
#include "render/OpenGLRenderInterface.h"
#include "utility/Stat.h"
#include "render/RenderMiscs.h"
#include <list>
#include "DecoSoftBody.h"
#include "ActiveSoftBody.h"
#include "utility/DecoLogger.h"
#include "utility/ConfigManager.h"
#include "collision/LDIImageGenerator.h"
#include "collision/CollisionHandler.h"
#include "collision/LCPFormulator.h"
#include "collision/ContactGroup.h"

DecoSceneNode::DecoSceneNode(DecoScene* scn, BOOL isLeaf, DecoSceneNode* lchild/* = NULL*/, DecoSceneNode* rchild/* = NULL*/)
: scene(scn)
, leftChild(NULL)
, rightChild(NULL)
, bIsLeaf(isLeaf)

{

	DecoLight* defaultLight = new DecoLight(vector3(0.10f, 0.4f, 0.45f), DecoColor(0x00afafafaf), DecoColor(0x00000000), LT_DirectionLight);

	sceneLights.push_back(defaultLight);



	DecoLight* defaultLight1 = new DecoLight(vector3(-0.0f, -0.14f, -0.45f), DecoColor(0x006f6f6f), DecoColor(0x00000000), LT_DirectionLight);

	sceneLights.push_back(defaultLight1);



	//DecoLight* defaultLight2 = new DecoLight(vector3(0.f, 2.f, 0.f), DecoColor(0x006ec1c2), DecoColor(0xffffffff), LT_PointLight);

	//defaultLight2->SetLightAttenuation(AT_Linear, 0.01f);

	//sceneLights.push_back(defaultLight2);



	//DecoLight* defaultLight2 = new DecoLight(vector3(0.f, 100.f, 0.f), DecoColor(0x000000ff), DecoColor(0x00000000), LT_SpotLight);

	//defaultLight2->SetSpotCutoffAngle(30);

	//defaultLight2->SetSpotDirection(vector3(-1.f, -1.f, 0.f));

	//defaultLight2->SetLightAttenuation(AT_Linear, 0.001f);

	//sceneLights.push_back(defaultLight2);



}

DecoSceneNode::~DecoSceneNode()

{

	//commented out by chenxz

	//leave memory management to DecoObjectFactory ba...



	//if (leftChild)

	//	delete leftChild;

	//if (rightChild)

	//	delete rightChild;

	//if (bIsLeaf)

	//{

	//	for (vector<DecoLight*>::iterator lightIter = sceneLights.begin(); lightIter != sceneLights.end(); lightIter++)

	//	{

	//		if (*lightIter)

	//		{

	//			delete *lightIter ;

	//			*lightIter = NULL;

	//		}

	//	}

	//	for (vector<DecoSceneObject*>::iterator objIter = sceneObjs.begin(); objIter != sceneObjs.end(); objIter++)

	//	{

	//		if (*objIter)

	//		{

	//			DecoObjectFactory::Destroy( *objIter );

	//			*objIter = NULL;

	//		}

	//	}

	//}

	//vector<DecoSceneObject*> sceneObjs;

	//vector<DecoLight*> sceneLights;

}



void DecoSceneNode::Render(DecoRenderInterface* RI, DecoCamera* camera, SceneObjectType objType, DecoDrawType drawType)

{

	if (bIsLeaf)
	{
		//RI->Clear(0, DecoColor(0x00000000), 1);



		RenderAllSceneObjects(RI, camera, objType, drawType);


		if (objType == AllT)
		{
			DecoSceneObject* focusObj = scene->GetFocusObject();

			if(focusObj != NULL)

			{

				RI->SetColor(0xffffffff);

				assert((focusObj)->GetRenderData());

				Box box = (focusObj)->GetRenderData()->GetBoundingBox();

				DecoRenderMisc::GetSingleton()->DrawBox(NULL, box);

			}
		}

	} 

	else

	{

		if (leftChild)

			leftChild->Render(RI, camera, objType, drawType);

		if (rightChild)

			rightChild->Render(RI, camera, objType, drawType);

	}

}

void DecoSceneNode::RenderAllSceneObjectsPlain(DecoRenderInterface* RI, DecoCamera* camera, SceneObjectType objType)
{
	for (vector<DecoSceneObject*>::iterator objIter = sceneObjs.begin(); objIter != sceneObjs.end(); objIter++)
	{
		DecoSceneObject* obj = *objIter;
		if (obj->IsRenderable() && obj->IsUpdatable() && (objType == AllT || obj->GetType() == objType))
		{
			DecoRenderData* objDecoRenderData = obj->GetRenderData();
			assert(objDecoRenderData);
			objDecoRenderData->Render(RI, NULL, 0, DT_SolidPolygon);
		}
	}

}

void DecoSceneNode::RenderAllSceneObjects(DecoRenderInterface* RI, DecoCamera* camera, SceneObjectType objType, DecoDrawType drawType)

{

	vector<DecoRenderData*> tempSortList;

	vector<DecoRenderData*> translucentDrawList;

	vector<DOUBLE> distanceList;

	ConvexVolume viewFrustum = camera->getViewFrustum();

	INT toRender = 0, beRendered = 0;

	DecoRenderData* floor = NULL, *ceiling = NULL;

	DecoLight* Lights[MAX_LIGHT];

	INT numEffectiveLights = 0;
	

	for (vector<DecoSceneObject*>::iterator objIter = sceneObjs.begin(); objIter != sceneObjs.end(); objIter++)

	{

		DecoSceneObject* obj = *objIter;

		if (obj->IsRenderable() && (objType == AllT || obj->GetType() == objType))

		{

			DecoRenderData* objDecoRenderData = obj->GetRenderData();

			assert(objDecoRenderData);

			//if (viewFrustum.BoxCheck(objDecoRenderData->GetBoundingBox()) != CF_Outside)

			{
				if (objDecoRenderData->NeedSort())

				{
					DOUBLE distSqr = (camera->getEye() - objDecoRenderData->GetBoundingBox().GetCenter()).lengthSqr();

					distanceList.push_back(distSqr);

					sort(distanceList.begin(), distanceList.end());

					vector<DOUBLE>::iterator it = find(distanceList.begin(), distanceList.end(), distSqr);

					translucentDrawList.insert(translucentDrawList.begin() + (it - distanceList.begin()), objDecoRenderData);

				}

				else

				{

					GetRelevantLights(objDecoRenderData, numEffectiveLights, Lights);

					objDecoRenderData->Render(RI, Lights, numEffectiveLights, drawType);

					DecoStat::GetSingleton()->CumulateObjectRendered(1);

				}



#ifdef DEBUG_BOUNDINGBOX

				RI->SetColor(0xff0000ff);

				objDecoRenderData->RenderBoundingBox(RI);

#endif
			}

			DecoStat::GetSingleton()->CumulateObjectTotal(1);

		}

	}


	for (vector<DecoRenderData*>::iterator translucentRenderDataIter = translucentDrawList.begin(); translucentRenderDataIter != translucentDrawList.end(); translucentRenderDataIter++)

	{

		GetRelevantLights((*translucentRenderDataIter), numEffectiveLights, Lights);

		(*translucentRenderDataIter)->RenderOpaqueSection(RI, Lights, numEffectiveLights, drawType);

		DecoStat::GetSingleton()->CumulateObjectRendered(1);

	}

	for (vector<DecoRenderData*>::iterator translucentRenderDataIter = translucentDrawList.begin(); translucentRenderDataIter != translucentDrawList.end(); translucentRenderDataIter++)

	{

		GetRelevantLights((*translucentRenderDataIter), numEffectiveLights, Lights);

		(*translucentRenderDataIter)->RenderTransparentSection(RI, Lights, numEffectiveLights, drawType);

	}

}



void DecoSceneNode::GetRelevantLights(const DecoRenderData* obj, INT& numLights, DecoLight** lights)

{

	numLights = 0;

	for (vector<DecoLight*>::iterator lightIter = sceneLights.begin(); lightIter != sceneLights.end(); lightIter++)

	{

		assert(numLights < MAX_LIGHT);

		lights[numLights] = *(lightIter);

		numLights++;

	}
}



DecoScene::DecoScene() : time(0), timeStep(0.03), mLDIWidth(0), mLDIHeight(0), mCollisionHandler(NULL), mIsCollisionDisabled(false)
#ifdef _LCP_CONTACT
, mLCPFormulator(NULL)
#endif
{

	RI = new DecoOpenGLRenderInterface();

	RI->Initialize();

	camera = new DecoCamera(RI);
	DecoRenderMisc::Initialize(RI);
	//only one node currently
	rootSceneNode = new DecoSceneNode(this, TRUE, NULL, NULL);
	DecoConfig::GetSingleton()->GetDouble("Scene", "timeStep", timeStep);
	const int defaultSceneBoxDim = 15; //20 by default
	SetSceneBox(Box(vector3(-defaultSceneBoxDim, -defaultSceneBoxDim, -defaultSceneBoxDim), vector3(defaultSceneBoxDim, defaultSceneBoxDim, defaultSceneBoxDim)));
    SetSpatialSubsivision(16, 16, 16);
    //SetSpatialSubsivision(4, 1, 2);

#ifdef _LCP_CONTACT
    mLCPFormulator = new LCPFormulator(this);
#endif
}

void DecoScene::SetGrid(const Box& box)
{
    mGrid.SetBoundingBox(box);
    
}

void DecoScene::SetSpatialSubsivision(int xGrids, int yGrids, int zGrids)
{
    mGrid.SetBoundingBox(scnBox);
    mGrid.SetResolution(xGrids, yGrids, zGrids);
}

const Grid& DecoScene::GetSpatialGrid() const
{
    return mGrid;
}

void DecoScene::SetTimeStep(double step)
{
	timeStep = step;
}

void DecoScene::DumpObjFiles(const std::string& fileName)
{
	vector <DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	for (vector<DecoSceneObject*>::iterator it = allObjs.begin(); it != allObjs.end(); ++it)
	{
		(*it)->GetRenderData()->DumpToObjFile(fileName);
	}
}

void DecoScene::PointScreen2WorldRay(const vector2& pt, vector3& outOrigin, vector3& outDir) const 

{

	vector3 viewVec = PointScreen2Camera(pt);

	matrix44 invview;

	RI->GetTransform(TT_WorldToCamera, invview);

	invview.invert();



	if (camera->isOrthogonal())

	{

		vector4 viewOrigin(viewVec);

		viewOrigin.z = 0.f;

		vector4 viewDir(0, 0, -1, 0);

		viewOrigin.w = 1.f;

		viewOrigin = invview * vector4(viewOrigin);

		vector4 worldVec = invview * viewDir;

		outOrigin = vector3(viewOrigin.x, viewOrigin.y, viewOrigin.z);

		outDir = vector3(worldVec.x, worldVec.y, worldVec.z);

	}

	else

	{

		viewVec.normalize();

		outOrigin = camera->getEye();	

		vector4 worldVec = invview * vector4(viewVec);

		outDir = vector3(worldVec.x, worldVec.y, worldVec.z);

	}
}




DecoColor Shading(const vector3& realIntersectPt, const vector3& norm, const vector3& lightPos, const vector3& viewPos,
				  const vector3& lightDiffuse, const vector3& lightSpecular, const vector3& lightAmbient, 
				  const vector3& matDiffuse, const vector3& matSpecular, const vector3&matAmbient)
{
	vector3 col(0, 0, 0);
	vector3 rayDir(lightPos - realIntersectPt);
	vector3 viewDir(viewPos - realIntersectPt);
	vector3 normal = norm;
	viewDir.normalize();
	rayDir.normalize();
	normal.normalize();
	vector3 halfDir = (viewDir + rayDir) / 2;
	halfDir.normalize();

	col += vector3(lightDiffuse.x * matDiffuse.x, lightDiffuse.y * matDiffuse.y, lightDiffuse.z * matDiffuse.z) * max(0.0, DotProduct(normal, rayDir));
	//col += vector3(lightAmbient.x * matAmbient.x, lightAmbient.y * matAmbient.y, lightAmbient.z * matAmbient.z);
	DOUBLE dotHalfAngle = max(0.0, DotProduct(halfDir, normal));
	col += vector3(lightSpecular.x * matSpecular.x, lightSpecular.y * matSpecular.y, lightSpecular.z * matSpecular.z) * dotHalfAngle * dotHalfAngle;
	return DecoColor(vector4(col.x, col.y, col.z, 1));
}

DecoColor Shading(const vector3& realIntersectPt, const vector3& norm, DecoLight** lights, INT numLights, const vector3& viewPos, DecoMaterial* mat)
{
	DecoColor col(0, 0, 0);
	vector3 matDiffuse, matSpecular, matAmbient;
	vector3 lightPos, lightDiffuse, lightSpecular, lightAmbient;
	if (mat)
	{
		matDiffuse = mat->GetDiffuse();
		matSpecular = mat->GetSpecular();
		matAmbient = mat->GetAmbient();
	}
	else
	{
		matDiffuse = vector3(1, 1, 1);
		matSpecular = vector3(0, 0, 0);
		matAmbient = vector3(0.5, 0.5, 0.5);
	}
	for (INT i = 0; i < numLights; i++)
	{
		lightPos = lights[i]->GetPosition();
		lightDiffuse = lights[i]->GetDiffuse();
		lightSpecular = lights[i]->GetSpecular();
		col += Shading(realIntersectPt, norm, lightPos,  viewPos, lightDiffuse, lightSpecular, lightAmbient, matDiffuse, matSpecular, matAmbient);
	}		
	lightAmbient = vector3(0.5, 0.5, 0.5);
	col += DecoColor(vector4(lightAmbient.x * matAmbient.x, lightAmbient.y * matAmbient.y, lightAmbient.z * matAmbient.z, 1));

	return col;
}

vector3 DecoScene::vectorScreen2World(const vector2& start, const vector2& end, DOUBLE startDistance, Plane constraint[], INT numConstraints) const

{

	if (!numConstraints)

	{

		vector3 viewVecStart = PointScreen2Camera(start);

		vector3 viewVecEnd = PointScreen2Camera(end);

		vector3 viewVec = viewVecEnd - viewVecStart;

		matrix44 invview;

		RI->GetTransform(TT_WorldToCamera, invview);

		invview.invert();

		vector4 worldVec = invview * vector4(viewVec);



		if (!camera->isOrthogonal())

		{

			DOUBLE nearD = camera->getNearDistance();

			DOUBLE nearDistance = sqrt(viewVec.lengthSqr() + nearD * nearD);

			if (startDistance == 0)

				startDistance = nearDistance;

			DOUBLE lengthRatio = startDistance / nearDistance;



			worldVec *= lengthRatio;

		}

		return vector3(worldVec.x, worldVec.y, worldVec.z);

	}

	else

	{

		assert(numConstraints == 1);

		vector3 originStart, originEnd, dirStart, dirEnd;

		PointScreen2WorldRay(start, originStart, dirStart);

		PointScreen2WorldRay(end, originEnd, dirEnd);

		vector3 intersectStart = constraint[0].RayIntersect(originStart, dirStart);

		vector3 intersectEnd = constraint[0].RayIntersect(originEnd, dirEnd);

		return (intersectEnd - intersectStart);

	}

}



vector3 DecoScene::PointScreen2Camera(const vector2& pt) const

{

	INT x, y, wid, ht;



	RI->GetViewport(x, y, wid, ht);

	vector2 canonicalPt(pt.x - (x + wid / 2), pt.y - (y + ht / 2));

	canonicalPt.x /= (wid / 2);

	canonicalPt.y /= (ht / 2);

	vector3 viewVec;

	float nearD = camera->getNearDistance();



	if (camera->isOrthogonal())

	{

		float width = camera->getWidth();

		float height = camera->getHeight();

		viewVec = vector3(canonicalPt.x * width / 2, canonicalPt.y * height / 2, -nearD);

	}

	else

	{

		float vAng = camera->getVerticalViewAngle();

		float aspRatio = static_cast<float>(wid) / ht;

		DOUBLE halfHeight = nearD * tan(vAng * PI / 360.f);

		DOUBLE halfWidth = aspRatio * halfHeight;



		viewVec = vector3(canonicalPt.x * halfWidth, canonicalPt.y * halfHeight, -nearD);

	}

	return viewVec;

}







DecoScene::~DecoScene()
{
	DestroyAll();
	if (camera)

	{
		delete camera;
		camera = NULL;
	}


	if (RI)

	{

		RI->Destroy();

		delete RI;

		RI = NULL;

	}
	if (mCollisionHandler)
	{
		delete mCollisionHandler;
		mCollisionHandler = NULL;
	}
#ifdef _LCP_CONTACT
    if (mLCPFormulator)
    {
        delete mLCPFormulator;
        mLCPFormulator = NULL;
    }
#endif
	mLDIWidth = mLDIHeight = 0;

}


void DecoScene::Render(BOOL bViewModel, BOOL bRenderAuxilary)

{

	//clear is done at caller, chenxz
	//RI->Clear();
	if (bViewModel)
   	    rootSceneNode->Render(RI, camera);
	if (bRenderAuxilary)
	{
		if (mCollisionHandler)
			mCollisionHandler->Render(RI);
		vector<DecoSceneObject*> allObjs;
		GetAllSceneObjects(allObjs);
		int numAllObjs = static_cast<int>(allObjs.size());
		for (int i = 0; i < numAllObjs; ++i)
		{
			if (allObjs[i]->GetType() == SoftT)
			{
				DecoSoftObject* softObj = static_cast<DecoSoftObject*>(allObjs[i]);
				softObj->DrawForce(RI);
			}
		}
        //mGrid.Render();
		DecoRenderMisc::GetSingleton()->DrawBox(rootSceneNode, scnBox);
	}

}

vector3 DecoScene::GetLDIPixelPosition(const LDIPixelTranslated& px, AXIS axis)
{	
	if (mLDIWidth == 0 || mLDIHeight == 0)
	{
		return vector3(0, 0, 0);
	}
	else
	{
		vector3 scnBoxExtent = scnBox.GetExtent();
		double depthScale = GetLDIPixelDepthScale(axis);
		float depth = 0;
		px.GetDepth(depth);
		switch (axis)
		{
		case AXIS_X:
			return vector3(depth * depthScale + scnBox[0].x, 
						   scnBoxExtent.y * 2 / mLDIHeight * (px.mRow + 0.5) + scnBox[0].y,
						   scnBoxExtent.z * 2 / mLDIWidth * (px.mCol + 0.5) + scnBox[0].z);
		case AXIS_Y:
			return vector3(scnBoxExtent.x * 2 / mLDIWidth * (px.mCol + 0.5) + scnBox[0].x, 
						   depth * depthScale + scnBox[0].y,
						   scnBoxExtent.z * 2 / mLDIHeight * (px.mRow + 0.5) + scnBox[0].z);
		case AXIS_Z:
			return vector3(-scnBoxExtent.x * 2 / mLDIWidth * (px.mCol + 0.5) + scnBox[1].x,
						   scnBoxExtent.y * 2 / mLDIHeight * (px.mRow + 0.5) + scnBox[0].y,
						   depth * depthScale + scnBox[0].z);
		}
	}
	return vector3(0, 0, 0);
}

DOUBLE DecoScene::GetLDIPixelArea(AXIS axis) const
{
	if (mLDIWidth == 0 || mLDIHeight == 0)
	{
		return 0;
	}
	else
	{
		vector3 scnBoxExtent = scnBox.GetExtent();
		switch (axis)
		{
		case AXIS_X:
			return (scnBoxExtent.z * 2 / mLDIWidth) * (scnBoxExtent.y * 2 / mLDIHeight);
		case AXIS_Y:
			return (scnBoxExtent.x * 2 / mLDIWidth) * (scnBoxExtent.z * 2 / mLDIHeight);
		case AXIS_Z:
			return (scnBoxExtent.x * 2 / mLDIWidth) * (scnBoxExtent.y * 2 / mLDIHeight);
		}
	}
	return 0;
}

DOUBLE DecoScene::GetLDIPixelDepthScale(AXIS axis) const
{
	if (mLDIWidth == 0 || mLDIHeight == 0)
	{
		return 0;
	}
	else
	{
		vector3 scnBoxExtent = scnBox.GetExtent();
		switch (axis)
		{
		case AXIS_X:
			return (scnBoxExtent.x * 2);
		case AXIS_Y:
			return (scnBoxExtent.y * 2);
		case AXIS_Z:
			return (scnBoxExtent.z * 2);
		}
	}
	return 0;
}

void DecoScene::RenderLDIModels(int width, int height, AXIS lookAxis, SoftObjRenderOption opt)
{
	RI->PushState();
	DecoCamera tmpCamera = *camera;

	mLDIWidth = width;
	mLDIHeight = height;

	RI->SetViewport(0, 0, width, height);

	float camWidth, camHeight, beginDepth, endDepth;

	switch (lookAxis)
	{
	case AXIS_X:
		camWidth = static_cast<float>(scnBox.GetExtent().z * 2);
		camHeight = static_cast<float>(scnBox.GetExtent().y * 2);
		beginDepth = static_cast<float>(scnBox[0].x - 1.0);
		endDepth = static_cast<float>(scnBox.GetExtent().x * 2);
		camera->set(vector3(beginDepth, scnBox.GetCenter().y, scnBox.GetCenter().z), vector3(beginDepth + 1.0, scnBox.GetCenter().y, scnBox.GetCenter().z), vector3(0, 1, 0));

		break;
	case AXIS_Y:
		camWidth = static_cast<float>(scnBox.GetExtent().x * 2);
		camHeight = static_cast<float>(scnBox.GetExtent().z * 2);
		beginDepth = static_cast<float>(scnBox[0].y - 1.0);
		endDepth = static_cast<float>(scnBox.GetExtent().y * 2);
		camera->set(vector3(scnBox.GetCenter().x, beginDepth, scnBox.GetCenter().z), vector3(scnBox.GetCenter().x, beginDepth + 1.0, scnBox.GetCenter().z), vector3(0, 0, 1));

		break;
	case AXIS_Z:
		camWidth = static_cast<float>(scnBox.GetExtent().x * 2);
		camHeight = static_cast<float>(scnBox.GetExtent().y * 2);
		beginDepth = static_cast<float>(scnBox[0].z - 1.0);
		endDepth = static_cast<float>(scnBox.GetExtent().z * 2);
		camera->set(vector3(scnBox.GetCenter().x, scnBox.GetCenter().y, beginDepth), vector3(scnBox.GetCenter().x, scnBox.GetCenter().y, beginDepth + 1.0), vector3(0, 1, 0));

		break;
	}
	camera->setOrtho(camWidth, camHeight, 1.f, endDepth + 1.f);	
	SetSoftObjectRenderOption(opt);
	rootSceneNode->RenderAllSceneObjectsPlain(RI, camera);


	*camera = tmpCamera;
	RI->PopState();
}

void DecoScene::DestroyAll()
{
	selectedObjs.clear();
	vector<DecoSceneObject* > & sceneObjs =  rootSceneNode->GetSceneObjects();
	for(vector<DecoSceneObject* >::iterator it = sceneObjs.begin();
		it != sceneObjs.end(); ++it)
		delete *it;
	sceneObjs.clear();
	if (rootSceneNode)
		delete(rootSceneNode);
}



BOOL DecoScene::EraseAllSceneObject()
{

	vector<DecoSceneObject* > & sceneObjs =  rootSceneNode->GetSceneObjects();
	int numObjs = static_cast<int>(sceneObjs.size());
	for (int i = 0; i < numObjs; ++i)
		delete sceneObjs[i];
	sceneObjs.clear();

	return TRUE;

}

void DecoScene::SelectMuscles(FLOAT x, FLOAT y, bool bAppend)
{
    DOUBLE depth = 1e30;
    ActiveSoftBody* selectedObj = NULL;
    int selectedMuscleId = -1;

    vector3 origin, dir;
    PointScreen2WorldRay(vector2(x, y), origin,  dir);
    vector<DecoSceneObject*> allObjs;
    GetAllSceneObjects(allObjs);
    for(vector<DecoSceneObject* >::iterator it = allObjs.begin(); it != allObjs.end(); ++it)
    {
        double rayDepth = -1.0;
        int muscleId = -1;
        DecoSceneObject* obj = *it;
        ActiveSoftBody* activeObj = dynamic_cast<ActiveSoftBody*> (obj);
        if (!activeObj)
        {
            LOG(INFO) << obj->GetName() << " is not an active softbody.";
            continue;
        }
        if (activeObj->SelectMuscles(origin, dir, rayDepth, muscleId))
        {
            if (rayDepth < depth)
            {
                depth = rayDepth;
                selectedMuscleId = muscleId;
                selectedObj = activeObj;
            }
        }
    }
    if (selectedObj)
        selectedObj->SetSelectMuscle(selectedMuscleId, bAppend);
}

void DecoScene::SelectPoints(FLOAT x, FLOAT y, bool bAppend)
{
	DOUBLE depth = 1e30;
	DecoSceneObject* selectedObj = NULL;
	int selectedVertexId = -1;

	vector3 origin, dir;
	PointScreen2WorldRay(vector2(x, y), origin,  dir);
	vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	for(vector<DecoSceneObject* >::iterator it = allObjs.begin(); it != allObjs.end(); ++it)
	{
		double rayDepth = -1.0;
		int vertexId = -1;
		DecoSceneObject* obj = *it;
		if (obj->SelectPoints(origin, dir, rayDepth, vertexId))
		{
			if (rayDepth < depth)
			{
				depth = rayDepth;
				selectedVertexId = vertexId;
				selectedObj = obj;
			}
		}
	}
	if (selectedObj)
		selectedObj->SetSelectVertex(selectedVertexId, bAppend);
}

DecoSceneObject* DecoScene::SelectObj(float x, float y)const

{

	DOUBLE depth = -1.f;

	vector3 origin, dir;

	PointScreen2WorldRay(vector2(x, y), origin,  dir);
	

	DecoSceneObject* retPtr = NULL;

	vector3 intersectionPT;

	//ASSERT(NULL != rootSceneNode);

	for(vector<DecoSceneObject* >::const_iterator it = rootSceneNode->GetSceneObjects().begin(); 
		it != rootSceneNode->GetSceneObjects().end(); ++it)
	{
		DecoSceneObject* obj = *it;
		if(obj->IsRenderable() && obj->IsSelectable())
		{	
			DOUBLE curdep = obj->GetRenderData()->GetBoundingBox().RayCheck(origin, dir);

			if(curdep > 0)
			{
				if (obj->GetRenderData()->RayIntersection(origin, dir, intersectionPT, curdep))
				{
					if (curdep < depth || depth < 0)
					{
						depth = curdep;
						retPtr = *it;
					}
				}
			}
		}
	}
	return retPtr;


}


void DecoScene::DeselectAll ()

{

	selectedObjs.erase(selectedObjs.begin(), selectedObjs.end());

}



BOOL DecoScene::IsSomethingSelected() const

{

	return (selectedObjs.size() > 0);

}


DecoSceneObject* DecoScene::GetFocusObject()const

{
	if(selectedObjs.size() > 0)

		return *(selectedObjs.begin());

	else

		return NULL;

}

DecoSceneObject* DecoScene::GetFirstObject()const
{
	return rootSceneNode->GetFirstSceneObj();
}

void DecoScene::SetFocusObject(DecoSceneObject* obj)

{

	selectedObjs.clear();

	if(obj != NULL)

        selectedObjs.push_back(obj);

}

void DecoScene::Destroy(DecoSceneObject* obj)
{
	if (!obj)
		return;
	for( vector<DecoSceneObject* >::iterator it = rootSceneNode->GetSceneObjects().begin(); 
		it != rootSceneNode->GetSceneObjects().end(); ++it)
	{
		if(*it == obj)
		{
			rootSceneNode->GetSceneObjects().erase(it);
			break;
		}
	}
	DecoSceneObject* focusObj = GetFocusObject();
	if (focusObj == obj)
		DeselectAll();
	delete obj;
}

void DecoScene::AddSceneObject(DecoSceneObject* solid)
{
	assert(solid && rootSceneNode);
	rootSceneNode->GetSceneObjects().push_back(solid);
}


DecoSceneObject* DecoScene::GetSceneObj(const char* name)
{
	vector<DecoSceneObject*> objs = rootSceneNode->GetSceneObjects();
	INT objSize = static_cast<INT>(objs.size());
	for (INT i = 0; i < objSize; i++)
	{
		DecoSceneObject* obj = objs[i];
		if (!strcmp(name, obj->GetName().c_str()))
			return obj;
	}
	return NULL;
}

void DecoScene::GetAllSceneObjects( vector<DecoSceneObject*> &allObjs)
{
	allObjs.clear();
	vector<DecoSceneObject*> objs = rootSceneNode->GetSceneObjects();
	allObjs.insert(allObjs.begin(), objs.begin(), objs.end());
}

DecoSoftObject* DecoScene::GetSoftObject(int index)
{
	vector<DecoSceneObject*> objs = rootSceneNode->GetSceneObjects();
	for (vector<DecoSceneObject*>::iterator it = objs.begin(); it != objs.end(); ++it)
	{
		if ((*it)->GetType() == SoftT)
		{
			DecoSoftObject* softObj = static_cast<DecoSoftObject*>(*it);
			if (softObj->GetID() == index)
				return softObj;
		}
	}
	return NULL;
}

const std::vector<CollisionInfo>& DecoScene::GetCollisionInfos()
{
	if (!mCollisionHandler)
	{
		mCollisionHandler = new CollisionHandler(this);
        if (!mIsCollisionDisabled)
		    mCollisionHandler->SetLDIResolution(128, 128);
	}
    if (!mIsCollisionDisabled) 
    {
        mCollisionHandler->DetectCollisions();
    }
	return mCollisionHandler->GetCollisionInfos();
}

void DecoScene::DisableCollisionDetection()
{
    mIsCollisionDisabled = true;
}
#ifdef _LCP_CONTACT

ContactGroup* DecoScene::GetContactGroup(DecoSceneObject* obj) const
{
    return mLCPFormulator->GetContactGroup(obj);
}

void DecoScene::GetContactForce(DecoSceneObject* obj, const VectorXd& normalMag, const VectorXd& tangentialMag, NodalForceT& normalForces, NodalForceT& frictionForces)
{
	mLCPFormulator->GetContactForce(obj, timeStep, normalMag, tangentialMag, normalForces, frictionForces);
}
void DecoScene::UpdateStage1(DOUBLE seconds)
{
    mLCPFormulator->UpdateStage1(seconds);
    if (!mIsCollisionDisabled)
	    updateContactPolygon();
}
void DecoScene::UpdateStage2(DOUBLE seconds, VectorXd* normalForce, VectorXd* frictionForce, VectorXd* lambda, bool bValidate)
{
    mLCPFormulator->UpdateStage2(seconds, normalForce, frictionForce, lambda, bValidate);
	if (!mIsCollisionDisabled)
		updateCenterOfPressure();
}

void DecoScene::GetInitialGuessForLCPSolver(ActiveSoftBody* body, vector<int>& isEqualityOnZ)
{
	ContactGroup* cg = GetContactGroup(body);
    if (cg)
    {
        body->PushForceAccumulator();
        body->SetDesiredMuscleLengthRelativeToRest();
        mLCPFormulator->GetInitialGuessForLCPSolver(timeStep, cg, isEqualityOnZ);
        body->PopForceAccumulator();
    }

}
#endif

void DecoScene::updateCenterOfPressure()
{
	vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	int numObjs = static_cast<int>(allObjs.size());

	const vector<CollisionInfo>& colInfos = mCollisionHandler->GetCollisionInfos();
	int numCollisionInfos = static_cast<int>(colInfos.size());

	for (int ithObj = 0; ithObj < numObjs; ++ithObj)
	{
		DecoSceneObject* obj = allObjs[ithObj];
		if (obj->GetType() != SoftT) continue;
		DecoSoftObject* softObj = static_cast<DecoSoftObject*>(obj);
		set<int> contactTriangleIds;
		set<int> contactNodeIds;
		for (int jthCol = 0; jthCol < numCollisionInfos; ++jthCol)
		{
			const CollisionInfo& colInfo = colInfos[jthCol];
			vector<int> contactIds = colInfo.GetContactTriangles(softObj);
			for (vector<int>::const_iterator it = contactIds.begin(); it != contactIds.end(); ++it)
				contactTriangleIds.insert(*it);

			contactIds = colInfo.GetContactNodes(softObj);
            for (vector<int>::const_iterator it = contactIds.begin(); it != contactIds.end(); ++it)
                contactNodeIds.insert(*it);
        }				
		if (!contactTriangleIds.empty())
			softObj->UpdateCenterOfPressure(contactTriangleIds, contactNodeIds);
	}
}

void DecoScene::updateContactPolygon()
{
	vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	int numObjs = static_cast<int>(allObjs.size());

	const vector<CollisionInfo>& colInfos = mCollisionHandler->GetCollisionInfos();
	int numCollisionInfos = static_cast<int>(colInfos.size());
	
	for (int ithObj = 0; ithObj < numObjs; ++ithObj)
	{
		DecoSceneObject* obj = allObjs[ithObj];
		ActiveSoftBody* activeObj = dynamic_cast<ActiveSoftBody*>(obj);
		if (!activeObj) continue;

		vector<int> contactNodeIds;
		for (int jthCol = 0; jthCol < numCollisionInfos; ++jthCol)
		{
			const CollisionInfo& colInfo = colInfos[jthCol];
			vector<int> contactIds = colInfo.GetContactNodes(activeObj);
			contactNodeIds.insert(contactNodeIds.end(), contactIds.begin(), contactIds.end());
		}				
		if (!contactNodeIds.empty())
			activeObj->UpdateContactPolygon(contactNodeIds);

	}
}


void DecoScene::Update(DOUBLE seconds)
{
	timeStep = seconds;
#ifdef _LCP_CONTACT
    UpdateStage1(seconds);
    UpdateStage2(seconds);

    //LCPFormulator lcp(this);
    //lcp.Update(seconds);
#else
	vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	DecoTimer timer;
	timer.startTimer();
	for (vector<DecoSceneObject*>::iterator objIter = allObjs.begin(); objIter != allObjs.end(); objIter++)
	{
		DecoSceneObject* obj = *objIter;
		if (obj->IsUpdatable())
 			obj->Update(seconds);
	}
	double duration = timer.getCurrentTime();
	VLOG(2) << "FEM Update used " << duration << " seconds.";
	timer.resetTimer();
	if (!mCollisionHandler)
	{
		mCollisionHandler = new CollisionHandler(this);
		mCollisionHandler->SetLDIResolution(128, 128);
	}

	mCollisionHandler->Update(seconds);	

	duration = timer.getCurrentTime();
	VLOG(2) << "Collision Update used " << duration << " seconds.";
#endif
	time += seconds;
}
DOUBLE DecoScene::GetCurrentTime()
{
	return time;
}

void DecoScene::ResetCurrentTime()
{
	time = 0;
}

void DecoScene::Reset()
{
	ResetCurrentTime();
	
}

void DecoScene::SetSoftObjectRenderOption(SoftObjRenderOption opt)
{
	std::vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	int numObjs = static_cast<int>(allObjs.size());
	for (int i = 0; i < numObjs; ++i)
	{
		if (allObjs[i]->GetType() == SoftT)
		{
			DecoSoftObject* softObj = static_cast<DecoSoftObject*>(allObjs[i]);
			softObj->SetRenderOption(opt);
		}
	}
}

void DecoScene::ToggleForceRenderOption(ForceRecorderRenderType type)
{
	std::vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	int numObjs = static_cast<int>(allObjs.size());
	for (int i = 0; i < numObjs; ++i)
	{
		if (allObjs[i]->GetType() == SoftT)
		{
			DecoSoftObject* softObj = static_cast<DecoSoftObject*>(allObjs[i]);
			softObj->ToggleForceRenderOption(type);
		}
	}
}

void DecoScene::SaveDeformedObjects(const string& filename)
{
	DecoArchive Ar(filename, AT_Write);

	std::vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	int numObjs = static_cast<int>(allObjs.size());
	Ar << numObjs;
	for (int i = 0; i < numObjs; ++i)
	{
		if (allObjs[i]->GetType() == SoftT)
		{
			allObjs[i]->Serialize(Ar);
		}
	}

}

void DecoScene::LoadDeformedObjects(const string& filename)
{
	EraseAllSceneObject();
	DecoSceneObject::gsObjId = 0;
	DecoArchive Ar(filename, AT_Read);


	int numObjs;
	Ar >> numObjs;
	for (int i = 0; i < numObjs; ++i)
	{
		DecoSceneObject* body = new DecoSoftObjectCorotationLinearFEM();
		body->Deserialize(Ar);
		AddSceneObject(body);
	}
}

void DecoScene::SaveDeformedObjectsShape()
{
	std::vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);
	int numObjs = static_cast<int>(allObjs.size());
	for (int i = 0; i < numObjs; ++i)
	{
		if (allObjs[i]->GetType() == SoftT)
		{
			DecoSoftObject* softObj = static_cast<DecoSoftObject*>(allObjs[i]);
			string filename = softObj->GetName();
			size_t pos = filename.find("models");
			filename.replace(pos, 6, "data");
			softObj->GetDeformedShapeTet()->WriteToFile(filename);
		}
	}

}

void DecoScene::MoveGrid(double x, double y, double z)
{
    mGrid.Move(x, y, z);
}

void DecoScene::MoveGridTo(double x, double y, double z)
{
    vector3 newCenter(x, y, z);
    vector3 oldCenter = mGrid.GetCenter();
    vector3 diff = newCenter - oldCenter;
    mGrid.Move(diff.x, diff.y, diff.z);
}
vector3 DecoScene::GetGridCenter() const
{
    return mGrid.GetCenter();
}

void DecoScene::MuteGravity()
{
    std::vector<DecoSceneObject*> allObjs;
    GetAllSceneObjects(allObjs);
    int numObjs = static_cast<int>(allObjs.size());
    for (int i = 0; i < numObjs; ++i)
    {
        allObjs[i]->MuteGravity();
    }
}

void DecoScene::GatherAndOutputStatistics(int ithSample1, int numSample1, double p1, int ithSample2, int numSample2, double p2)
{
	static bool bFirstTime = true;


	bFirstTime = false;
}


void DecoScene::RecordMovie(DecoArchive& Ar, double playerVersion)
{
    vector<DecoSceneObject*> allObjs;
    GetAllSceneObjects(allObjs);
    int numObjs = static_cast<int>(allObjs.size());
    for (int i = 0; i < numObjs; ++i)
    {
        allObjs[i]->RecordMovie(Ar, playerVersion);
    }
}


void DecoScene::ClearContactForce()
{
	vector<DecoSceneObject*> allObjs;
	GetAllSceneObjects(allObjs);

	for (std::vector<DecoSceneObject*>::iterator objIter = allObjs.begin(); objIter != allObjs.end(); objIter++)
	{
		DecoSceneObject* obj = *objIter;
		if (obj->GetType() == SoftT)
		{
			DecoSoftObject* softObj = static_cast<DecoSoftObject*>(obj);
			softObj->ClearContactForce();
		}
	}

}

void DecoScene::Serialize(DecoArchive& Ar)
{
	Ar << (*rootSceneNode);
	Ar << scnBox;
	Ar << time;
	Ar << *(static_cast<DecoOpenGLRenderInterface*>(RI)) << (*camera);

}
void DecoScene::Deserialize(DecoArchive& Ar)
{
	Ar >> (*rootSceneNode);
	Ar >> scnBox;
	Ar >> time;
	Ar >> *(static_cast<DecoOpenGLRenderInterface*>(RI)) >> (*camera);

	DecoConfig::GetSingleton()->GetDouble("Scene", "timeStep", timeStep);

	SetSceneBox(scnBox);

}




DecoArchive& operator<< (DecoArchive& Ar, const DecoSceneNode& node)
{
	Ar << node.bIsLeaf;
	INT objNum = static_cast<INT>(node.sceneObjs.size());
	Ar << objNum;
	for (INT i = 0; i < objNum; i++)
	{
	    SceneObjectType type = node.sceneObjs[i]->GetType();
		Ar << static_cast<INT>(type);
		switch(type)
		{
		case SoftT:	
			node.sceneObjs[i]->Serialize(Ar);
			break;
		default:
			break;
		}
	}
	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, DecoSceneNode& node)
{
	Ar >> node.bIsLeaf;
	UINT objNum = 0;
	Ar >> objNum;
	for (UINT i = 0; i < objNum; i++)
	{
		INT type = 0;
		Ar >> type;
		if (type == SoftT)
		{
			DecoSoftObject* obj = new DecoSoftObject();
			obj->Deserialize(Ar);
			node.sceneObjs.push_back(obj);
		}
	}

	return Ar;
}
