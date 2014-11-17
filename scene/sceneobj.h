#ifndef SCENE_OBJ_H
#define SCENE_OBJ_H

#include "stdafx.h"

#include "render/RenderInterface.h"
#include "render/RenderType.h"
#include <string>
#include "StaticMesh.h"
#include "collision/ContactPolygon.h"

using namespace std;

class RenderableSceneObject;
class DecoRenderData;


typedef enum
{
	FluidT = 0,
	SolidT = 1,
	SprayerT = 2,
	FreerT = 3,
	ArticulatedT = 4,
	SoftT = 5,
	AllT = 6,
	UndefinedT = 7,
} SceneObjectType;

enum DecoOpFlag
{
	OpRenderable,
	OpMovable,
	OpRotatable,
	OpSelectable,
	OpScalable,
	OpReflectable,
	OpTransparentable
};

class DecoSceneObject
{
protected:
	BOOL bIsRenderable;
	BOOL bIsMovable;
	BOOL bIsRotatable;
	BOOL bIsSelectable;
	BOOL bIsScalable;
	BOOL bIsReflectable;
	BOOL bIsTransparentable;
	BOOL bIsUpdatable;
    BOOL bIsGravityMuted;

	INT objID;
	string name;
	SceneObjectType objType;

	//added by chenxz//
	Coords coords;
	DecoStaticMesh mesh;
	vector<int> mSelectedVertices;
	vector<int> mContactVertices;
	ContactPolygon mContactPolygon;
	///////////////////
public:
	static int gsObjId;
	DecoSceneObject()
					: objType(SoftT),
					bIsRenderable(FALSE),
					bIsMovable(FALSE),
					bIsRotatable(FALSE),
					bIsSelectable(FALSE),
					bIsScalable(FALSE),
					bIsReflectable(FALSE),
					bIsTransparentable(FALSE),
					bIsUpdatable(FALSE),
                    bIsGravityMuted(FALSE),
					objID(-1),
					name("undefined")
	{}
	DecoSceneObject(SceneObjectType type)
				  : objType(type),
				    bIsRenderable(FALSE),
					bIsMovable(FALSE),
					bIsRotatable(FALSE),
					bIsSelectable(FALSE),
					bIsScalable(FALSE),
					bIsReflectable(FALSE),
					bIsTransparentable(FALSE),
					bIsUpdatable(FALSE),
                    bIsGravityMuted(FALSE),
					objID(-1),
					name("undefined")
	{}
	virtual ~DecoSceneObject() {}

	virtual BOOL CreateSceneObjectFromMesh(const std::string & path, double timeStep = 0.0333);
	BOOL CreateSceneObjectFromMesh(const DecoStaticMesh* mesh, double timeStep = 0.0333);
	virtual void Serialize(DecoArchive& Ar) const;
	virtual void Deserialize(DecoArchive& Ar);

public:

	Coords& GetCoords()
	{
		return coords;
	}
	void SetCoords(const Coords& newCoords)
	{
		coords = newCoords;
	}
    INT GetID()const
	{
		return objID;
	}
	SceneObjectType GetType()const
	{
		return objType;
	}
	void DynamicChangeFlag(DecoOpFlag flag, BOOL value);

	BOOL IsSelectable () const
	{
		return bIsSelectable;
	}
	BOOL IsRenderable () const
	{
		return bIsRenderable;
	}
	BOOL IsMovable () const
	{
		return bIsMovable;
	}
	BOOL IsRotatable () const
	{
		return bIsRotatable;
	}
	BOOL IsReflectable () const
	{
		return bIsReflectable;
	}
	BOOL IsTransparentable () const
	{
		return bIsTransparentable;
	}
	BOOL IsUpdatable() const
	{
		return bIsUpdatable;
	}
	const std::string & GetName()const
	{
		return name;
	}
	void SetName(const std::string& newname)
	{
		name = newname;
	}
    virtual void RecordMovie(DecoArchive& Ar, double playerVersion) const
    {
    }
	virtual void CacheMovie(DecoArchive& Ar, double playerVersion)
	{

	}
	virtual void PlaybackMovie(int ithFrame)
	{

	}
	virtual void SetInitialPoseForMovie(DecoArchive& Ar)
	{

	}
	//added by chenxz//
	//inherited sceneobjs override following methods to get customized
	virtual DecoRenderData* GetRenderData()
	{
		mesh.SetLocalToWorld(coords.GetMatrix());
		return &mesh;
	};
    virtual void MuteGravity()
    {
        bIsGravityMuted = TRUE;
    }
    virtual Vector3d GetCenterOfMass() const
    {
        return Vector3d::Zero();
    }
	virtual BOOL Move(const vector3& offset);
	virtual BOOL Rotate(AXIS axis, float rad, BOOL bAlongGlobal = TRUE);
	virtual BOOL RotateAlongGlobalVector(vector3 axis, FLOAT rad);
	virtual BOOL Scale(const vector3& fac);
	virtual BOOL SetPosition(const vector3& pos);
	virtual vector3 SampleVelocity(const vector3& pos);
	virtual void Update(DOUBLE deltaTime);
	virtual void SetSelectVertex(int id, bool bAppend);
	virtual bool SelectPoints(const vector3& origin, const vector3& dir, double& depth, int& vertexId);
	virtual void UpdateContactPolygon();
	virtual const ContactPolygon& GetContactPolygon() const;
	virtual const vector<int>& GetContactPoints() const;
	virtual void RenderContactPolygon(DecoRenderInterface* RI);
	
};

#endif