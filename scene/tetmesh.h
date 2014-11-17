#ifndef _TET_MESH
#define _TET_MESH

#include "tetgen.h"
#include "utility/mathlib.h"
#include "render/RenderInterface.h"

class DecoSceneObject;

enum SoftObjRenderOption
{
	RO_Points,
	RO_Edges,
	RO_Mesh,
	RO_Tetrahedra,
	RO_ID,
	RO_Barycentric,
};


class Tetrahedron
{
public:
	Tetrahedron();
	Tetrahedron(const vector3& a, const vector3& b, const vector3& c, const vector3& d);
	double CalculateVolume() const;
	double CalculateSignVolume() const;
	vector3 CalculateCenter() const;
	bool PointInside(const vector3& pt) const;
	vector3 BaryCentricCoord(const vector3& pt) const;
	vector3 mVertices[4];
	int mVertIndices[4];

};

class TetMesh : public DecoRenderData
{
public:
	TetMesh();
	~TetMesh();
	vector3 GetIthVertex(int i) const;
	void SetIthVertex(const vector3& pt, int i);
	void GetVertexIndicesForIthTriangle(int ithTriangle, int& indexA, int& indexB, int& indexC);
	int GetNumVertices() const;
	int GetNumTetrahedra() const;
	Tetrahedron GetIthTetrahedron(int i) const;
	bool LoadFromFile(const string& fileName);
	void WriteToFile(const string& fileName);
	void SetRenderOption(SoftObjRenderOption option);

	virtual Box GetBoundingBox();
	virtual Box	GetUntransformedBoundingBox() const;
	virtual void SetLocalToWorld(const matrix44& matrix);
	virtual void RenderBoundingBox(DecoRenderInterface* RI);
	virtual BOOL RayIntersection(const vector3& origin, const vector3& dir, vector3& intersectionPt, DOUBLE& time, vector3* normal = NULL);
	virtual BOOL NeedSort();
	virtual void Render(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	virtual void RenderTransparentSection(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	virtual void RenderOpaqueSection(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	virtual VOID Scale(DOUBLE xcoef, DOUBLE ycoef, DOUBLE zcoef);
	virtual int RayIntersectionCount(const vector3& origin, const vector3& dir);
	virtual BOOL IsPrimitiveInBox(Box& box);
	virtual void RetrieveAllVertices(vector<vector3>& allVertices);
	virtual BOOL IsPointInside(const vector3& pt, bool bVote = false);
	virtual void DumpToObjFile(const string& fileName);
	virtual void SetID(int id);
	virtual void SetObject(DecoSceneObject* obj);
	virtual void SetPointColor(const vector<DecoColor>& colors);
	virtual void SetColor(const DecoColor& color);
	void SetSelectPoint(const vector<int> indices);
    void SetCenterOfMass(const Vector3d& com);
	void SetCenterOfPressure(const Vector3d& cop);
protected:
	DecoSceneObject* mObj;
	tetgenio mTetGenIO;
	Box mbb;
	SoftObjRenderOption mRenderOption;
	int mObjectID;
	DecoMaterial mMaterial;
	vector<int> mSelectedPoints;
    Vector3d mCenterOfMass;
	Vector3d mCenterOfPressure;
	vector<DecoColor> mPointColors;
	void calculateVertices(std::vector<vector3>& vertices);
	void calculateNormals(std::vector<vector3>& normals);
	void calculateIDColors(std::vector<DecoColor>& colors);
	void calculateBarycentricColors(std::vector<DecoColor>& colors);
	void renderSelected(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderSelectedPoints(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderPoints(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderEdges(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderMesh(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderTetrahedra(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderTriangleID(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderBarycentricCoord(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
    void renderCenterOfMass(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderContactPolygon(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
	void renderCenterOfPressure(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon);
};
#endif