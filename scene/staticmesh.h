#ifndef STATIC_MESH_H
#define STATIC_MESH_H

#include "stdafx.h"
#include <string>
#include <vector>
using namespace std;
#include "render/RenderInterface.h"
#include "render/RenderResource.h"
//#include "abUtil.H"


class DecoStaticMesh;

class DecoStaticMeshSection
{
	friend class DecoStaticMesh;
private:
	DecoVertexBuffer			*m_vb;
	DecoIndexBuffer				*m_ib;				 // Index Buffer, keep it for future use, who knows...	
	size_t						m_numPrimitives;	 // Number of primitives
	DecoPrimitiveType			m_primType;			 // Primitive type
	DecoStaticMesh				*m_pStaticMesh;      // pointer to this section's father
	Box							m_boundingBox;

	FLOAT						m_pre_u_tiling;
	FLOAT						m_pre_v_tiling;

	VOID					    AdjustTexCoords();
	VOID						Clear();
	VOID						CopyFrom(const DecoStaticMeshSection& rhs);

public:
	DecoStaticMeshSection( DecoStaticMesh* pStaticMesh )
		: m_vb(NULL), m_ib(NULL), m_pre_u_tiling(1.0f), m_pre_v_tiling(1.0f),
		m_numPrimitives(0),  m_primType(PT_TriangleList)
	{
		assert(pStaticMesh);
		m_pStaticMesh = pStaticMesh;
	}

	DecoStaticMeshSection(const DecoStaticMeshSection&);
	~DecoStaticMeshSection();
	DecoStaticMeshSection& operator=(const DecoStaticMeshSection&);

	VOID Init(const ASE::GeomObject& obj);

	BOOL ReadFromFile(ifstream& inFile);
	VOID WriteToFile(ofstream& outFile);
	INT GetMaterialRef() const
	{
		if (m_vb)
			return m_vb->GetMaterialRef();
		else 
			return -1;
	}
	VOID SetNumPrimitives(size_t num)
	{
		m_numPrimitives = num;
	}

	VOID SetPrimitiveType(DecoPrimitiveType type)
	{
		m_primType = type;
	}

	VOID SetVertexBuffer(DecoVertexBuffer* vb)
	{
		m_vb = vb;
	}

	VOID SetIndexBuffer(DecoIndexBuffer* ib)
	{
		m_ib = ib;
	}

	Box BoundingBox()
	{
		return m_boundingBox;
	}

	VOID Scale(DOUBLE coef);
	VOID Scale(DOUBLE xcoef, DOUBLE ycoef, DOUBLE zcoef);
	VOID Render(DecoRenderInterface* RI, DecoDrawType DrawType = DT_SolidPolygon);
	VOID SetTextureRepeat(FLOAT uRepeat, FLOAT vRepeat);
	BOOL RayIntersection(const vector3& origin, const vector3& dir, vector3& intersectionPt, DOUBLE& time, vector3* normal);
	int RayIntersectionCount(const vector3& origin, const vector3& dir);
	VOID ExchangeYZAxis();
	VOID CenterModel(vector3 center);
	BOOL NeedSort ();
	BOOL IsPrimitiveInBox(const Box& box);
	void RetrieveAllVertices(vector<vector3>& allVertices);
	void RetrieveAllNormals(vector<vector3>& allNormals);

};

class SmoothGroupInfo
{
public:
	int mGroupId;
	int mStartFaceIdx;
};

class DecoStaticMesh : public DecoRenderData
{
	friend class DecoStaticMeshSection;
private:
	vector<DecoStaticMeshSection*>  m_sections;	
	vector<DecoMaterial*>			m_materials;
	size_t							m_num_sections;
	size_t							m_num_materials;

	matrix44						m_localToWorld;
	Box								m_boundingBox;
	std::vector<SmoothGroupInfo>    m_smoothGroupInfos;

	VOID	Clear();
	VOID	CopyFrom(const DecoStaticMesh&);
	void readObj(const char* fname, vector<vector3>& vertices, vector<Index3INT>& faces);
	void computeNormals(const vector<vector3>& vertices, const vector<Index3INT>& faces, vector<vector3>& norms);
	void computeColor(const vector<vector3>& vertices, const vector<Index3INT>& faces, vector<DecoColor>& cols);

public:
	DecoStaticMesh();

	DecoStaticMesh(const DecoStaticMesh&);
	~DecoStaticMesh();
	DecoStaticMesh& operator=(const DecoStaticMesh&);
	VOID	ReadFromFile(const string& fileName);
	VOID	ReadFromObjFile(const string& fileName);
	void	DumpToObjFile(const string& fileName);
	virtual BOOL IsPointInside(const vector3& pt, bool bUseVote = false);
	virtual void Render(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	virtual void RenderTransparentSection(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	virtual void RenderOpaqueSection(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);

	//VOID	Render(DecoRenderInterface* RI, DecoSceneNode* SceneNode = NULL, DecoDrawType DrawType = DT_SolidPolygon);
	//VOID	RenderTransparentSection(DecoRenderInterface* RI, DecoSceneNode* SceneNode = NULL, DecoDrawType DrawType = DT_SolidPolygon);
	//VOID	RenderOpaqueSection(DecoRenderInterface* RI, DecoSceneNode* SceneNode = NULL, DecoDrawType DrawType = DT_SolidPolygon);

	Box		GetBoundingBox();
	Box		GetUntransformedBoundingBox() const ;
	VOID	RenderBoundingBox(DecoRenderInterface* RI);
	VOID	SetLocalToWorld(const matrix44& matrix);
	matrix44 GetLocalToWorld() const;
	//VOID	CreateTestInstance();
	//VOID	CreateStaticMesh(const Box& box);
	//VOID	CreateCeilingFloorFromPlane(const vector2& min, const vector2& max, FLOAT height, BOOL bFloor, FLOAT relfectivity = 0.f);

	//dim: x 宽度 y 高度 z厚度
	//norm: 与norm垂直的面需要贴图
	void	CreateWindowDoorFromBox(const Box& dim, const vector3& norm);

	BOOL	ReadFromFile(ifstream& inFile);
	VOID	WriteToFile(ofstream& outFile);
	
	// return the material's index after been added.
	size_t	AddMaterial(DecoMaterial* material);
	// return the static mesh section's index after been added.
	size_t	AddStaticMeshSection(DecoStaticMeshSection* section);

	// return the pointer to material
	DecoMaterial* GetMaterial(size_t index);
	DecoMaterial* GetFirstValidMaterial();
	// return the pointer to static mesh section
	DecoStaticMeshSection* GetStaticMeshSection(size_t index);

	// Exchange Y and Z axis, then move the model to center it at origin, in
	// order to rotate it and place it
	VOID	ExchangeYZAxis();
	VOID	CenterModel();

	VOID	SetTextureRepeat(FLOAT uRepeat, FLOAT vRepeat);
	BOOL	RayIntersection(const vector3& origin, const vector3& dir, vector3& intersectionPt, DOUBLE& time , vector3* normal = NULL);
	BOOL	NeedSort();
	// temp function
	VOID	Scale(DOUBLE coef);
	VOID	Scale(DOUBLE xcoef, DOUBLE ycoef, DOUBLE zcoef);
	void ScaleTo(DOUBLE xcoef, DOUBLE ycoef, DOUBLE zcoef);

	int RayIntersectionCount(const vector3& origin, const vector3& dir);
	BOOL IsPrimitiveInBox(Box& box);
	void RetrieveAllVertices(vector<vector3>& allVertices);
	void RetrieveAllNormals(vector<vector3>& allNormals);
	//void GetAbVextexInformation(AbVecList& vetexList, AbTriList& faceList, AbVecList& normList);
	void CreateFromVertexBuffer(DecoVertexBuffer* vb);
	void CreateFromVertexBuffer(DecoVertexBuffer** vb, INT numVB);
	DOUBLE PointDistance(const vector3& pt);
	void DumpRibFile(const string& filename, BOOL bAppend = FALSE);
private:
	VOID	CalculateBoundingBox();
};


#endif
