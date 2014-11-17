#include "utility/ConfigManager.h"
#include "SceneObj.h"

int DecoSceneObject::gsObjId = 0;

void DecoSceneObject::DynamicChangeFlag(DecoOpFlag flag, BOOL value)
{
	switch (flag)
	{
	case OpRenderable:
		bIsRenderable = value;
		break;
	case OpMovable:
		bIsMovable = value;
		break;
	case OpRotatable:
		bIsRotatable = value;
		break;
	case OpSelectable:
		bIsSelectable = value;
		break;
	case OpScalable:
		bIsScalable = value;
		break;
	case OpReflectable:
		bIsReflectable = value;
		break;
	case OpTransparentable:
		bIsTransparentable = value;
		break;
	}
}

void DecoSceneObject::Serialize(DecoArchive& Ar) const
{
	Ar << name;
	Ar << bIsRenderable;
	Ar << bIsMovable;
	Ar << bIsRotatable;
	Ar << bIsSelectable;
	Ar << bIsScalable;
	Ar << bIsReflectable;
	Ar << bIsTransparentable;
	Ar << bIsUpdatable;
	Ar << objID;
	Ar << objType;
	Ar << coords;
}
void DecoSceneObject::Deserialize(DecoArchive& Ar)
{	
	Ar >> name;

	//char realPath[128];

	//string modelPath;
	//DecoConfig::GetSingleton()->GetString("Path", "modelPath", modelPath);
	//sprintf(realPath, "%s%s.ase", modelPath.c_str(), name.c_str());
	CreateSceneObjectFromMesh(name);
	Ar >> bIsRenderable;
	Ar >> bIsMovable;
	Ar >> bIsRotatable;
	Ar >> bIsSelectable;
	Ar >> bIsScalable;
	Ar >> bIsReflectable;
	Ar >> bIsTransparentable;
	Ar >> bIsUpdatable;
	Ar >> objID;
	Ar >> (INT&)objType;
	Ar >> coords;
}


BOOL DecoSceneObject::CreateSceneObjectFromMesh(const std::string & path, double timeStep)
{		
	//INT startPos = 0, endPos = 0;

	//for (INT i = static_cast<INT>(path.length()) - 1; i >= 0; i--)
	//{
	//	if (path[i] == '.')
	//	{
	//		endPos = i;
	//	}
	//	if (path[i] == '\\')
	//	{
	//		startPos = i + 1;		
	//		if (endPos > startPos)
	//			break;
	//	}

	//}
	//string realName = path.substr(startPos, endPos - startPos);
	SetName(path);
	coords.SetOrigin(vector3(0,0,0));


	char lowerpath[256];
	size_t i;
	for(i = 0; i < path.length(); ++i)
	{
		lowerpath[i] = tolower(path[i]);
	}
	lowerpath[i] = '\0';
	char *c = NULL;
	if((c = strstr(lowerpath, ".ase")) > 0)
	{
		ifstream inFile(path.c_str(), ios::in);
		if(inFile.fail())
	{
			c[0] = '.';
			c[1] = 'o';
			c[2] = 'b';
			c[3] = 'j';
		}
		else
		mesh.ReadFromFile(path);
	}
	if (strstr(lowerpath, ".obj") > 0)
	{
		mesh.ReadFromObjFile(lowerpath);
		Rotate(AXIS_X, static_cast<FLOAT>(PI / 2));
	}
	if(strstr(lowerpath, ".xjj") > 0)
	{
		ifstream inFile(path.c_str(), ios::in | ios::binary);
		if(inFile.fail())
		{
			return FALSE;
		}
		mesh.ReadFromFile(inFile);
	}
	//else if (strstr(lowerpath, ".lzma") > 0)
	//{
	//	string target = path.substr(0, path.size() - 4) + "lxjj";
	//	DecodeFile(path.c_str(), target.c_str());
	//	ifstream inFile(target.c_str(), ios::in | ios::binary);
	//	if(inFile.fail())
	//	{
	//		return FALSE;
	//	}
	//	mesh.ReadFromFile(inFile);
	//}

	return TRUE;
}

BOOL DecoSceneObject::SetPosition(const vector3& pos)
{
	if(!bIsMovable)
		return FALSE;
	coords.SetOrigin(pos);
	return TRUE;
}


BOOL DecoSceneObject::Scale(const vector3& fac)
{
	mesh.Scale(fac.x, fac.y, fac.z);
	return TRUE;
}

BOOL DecoSceneObject::Move(const vector3& offset)
{
	coords.Translate(offset);
	mesh.SetLocalToWorld(coords.GetMatrix());

	return TRUE;
}
BOOL DecoSceneObject::Rotate(AXIS axis, float rad, BOOL bAlongGlobal/* = TRUE*/)
{
	if(bAlongGlobal)
		coords.RotateAlongGlobalAxis(rad, axis);
	else
		coords.RotateAlongLocalAxis(rad, axis);
	mesh.SetLocalToWorld(coords.GetMatrix());

	return TRUE;
}

BOOL DecoSceneObject::RotateAlongGlobalVector(vector3 axis, FLOAT rad)
{
	coords.RotateAlongGlobalVector(rad, axis);
	mesh.SetLocalToWorld(coords.GetMatrix());
	return TRUE;
}
vector3 DecoSceneObject::SampleVelocity(const vector3& pos)
{
	return vector3(0, 0, 0);
}

void DecoSceneObject::Update(DOUBLE deltaTime)
{

}

void DecoSceneObject::UpdateContactPolygon()
{

}
void DecoSceneObject::RenderContactPolygon(DecoRenderInterface* RI)
{
	mContactPolygon.Render(RI);
}

const ContactPolygon& DecoSceneObject::GetContactPolygon() const
{
	return mContactPolygon;
}

const vector<int>& DecoSceneObject::GetContactPoints() const
{
	return mContactVertices;
}

BOOL DecoSceneObject::CreateSceneObjectFromMesh(const DecoStaticMesh* mesh, double timeStep)
{
	SetName("TetrahedraMesh");
	coords.SetOrigin(vector3(0,0,0));
	this->mesh = *mesh;

	return TRUE;
}

void DecoSceneObject::SetSelectVertex(int id, bool bAppend)
{
	if (!bAppend)
		mSelectedVertices.clear();
	mSelectedVertices.push_back(id);
}

bool DecoSceneObject::SelectPoints(const vector3& origin, const vector3& dir, double& depth, int& vertexId)
{
	return false;
}