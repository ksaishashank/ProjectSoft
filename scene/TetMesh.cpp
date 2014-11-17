#include "render/RenderType.h"
#include "TetMesh.h"
#include "scene/sceneObj.h"
#include "scene/ActiveSoftBody.h"
#include <Eigen/Dense>
using namespace Eigen;

Tetrahedron::Tetrahedron()
{

}

Tetrahedron::Tetrahedron(const vector3& a, const vector3& b, const vector3& c, const vector3& d)
{
	mVertices[0] = a;
	mVertices[1] = b;
	mVertices[2] = c;
	mVertices[3] = d;
}

vector3 Tetrahedron::CalculateCenter() const
{
	return (mVertices[0] + mVertices[1] + mVertices[2] + mVertices[3]) / 4.0;
}

double Tetrahedron::CalculateVolume() const
{
	return abs(CalculateSignVolume());
}

double Tetrahedron::CalculateSignVolume() const
{
	vector3 v01 = mVertices[1] - mVertices[0];
	vector3 v02 = mVertices[2] - mVertices[0];
	vector3 v03 = mVertices[3] - mVertices[0];
	vector3 cross = CrossProduct(v02, v03);
	double mixed = DotProduct(v01, cross);
	double volume = mixed / 6.0;
	return volume;
}

bool Tetrahedron::PointInside(const vector3& pt) const
{
	double w = CalculateSignVolume();
	Tetrahedron sTet(mVertices[0], pt, mVertices[2], mVertices[3]);
	Tetrahedron uTet(mVertices[0], mVertices[1], pt, mVertices[3]);
	Tetrahedron tTet(mVertices[0], mVertices[1], mVertices[2], pt);
	double s = sTet.CalculateSignVolume();
	double u = uTet.CalculateSignVolume();
	double t = tTet.CalculateSignVolume();
	if (abs(s + u + t) <= abs(w) && w * s >= 0 && w * u >= 0 && w * t >= 0)
		return true;
	else
		return false;
}

vector3 Tetrahedron::BaryCentricCoord(const vector3& pt) const
{
	vector3 da = mVertices[0] - mVertices[3];
	vector3 db = mVertices[1] - mVertices[3];
	vector3 dc = mVertices[2] - mVertices[3];
	vector3 dp = pt - mVertices[3];
	Matrix3d lhs;
	lhs << da.x, db.x, dc.x,
		   da.y, db.y, dc.y,
		   da.z, db.z, dc.z;
	Vector3d rhs;
	rhs << dp.x, dp.y, dp.z;
	Vector3d coord = lhs.colPivHouseholderQr().solve(rhs);
	CHECK(coord[0] >= -1e-6 && coord[1] >= -1e-6 && coord[2] >= -1e-6 && coord.sum() <= 1.0 + 1e-6);
	vector3 reconstructPt = coord[0] * mVertices[0] + coord[1] * mVertices[1] + coord[2] * mVertices[2] + (1.0 - coord.sum()) * mVertices[3];
	CHECK_NEAR(reconstructPt.x, pt.x, 1e-6);
	CHECK_NEAR(reconstructPt.y, pt.y, 1e-6);
	CHECK_NEAR(reconstructPt.z, pt.z, 1e-6);
	return vector3(coord[0], coord[1], coord[2]);
}


TetMesh::TetMesh() : mRenderOption(RO_Mesh), mObj(NULL)
{
	mMaterial.SetDiffuse(vector4(1.0, 1.0, 1.0, 1.0));
}
TetMesh::~TetMesh()
{

}

void TetMesh::SetObject(DecoSceneObject* obj)
{
	mObj = obj;
}


void TetMesh::SetColor(const DecoColor& color)
{
	mMaterial.SetDiffuse(color.ToVec4());
}

void TetMesh::GetVertexIndicesForIthTriangle(int ithTriangle, int& indexA, int& indexB, int& indexC)
{
	assert(ithTriangle < mTetGenIO.numberoftrifaces);
	indexA = mTetGenIO.trifacelist[3 * ithTriangle];
	indexB = mTetGenIO.trifacelist[3 * ithTriangle + 1];
	indexC = mTetGenIO.trifacelist[3 * ithTriangle + 2];
}

vector3 TetMesh::GetIthVertex(int i) const
{
	if (i >= mTetGenIO.numberofpoints || i < 0)
	{
		std::cout<<"index " << i << "out of range " << mTetGenIO.numberofpoints << "in TetMesh::GetIthVertex" << std::endl;
		return vector3(0, 0, 0);
	}
	return vector3(mTetGenIO.pointlist[3 * i], mTetGenIO.pointlist[3 * i + 1], mTetGenIO.pointlist[3 * i + 2]);
}

void TetMesh::SetIthVertex(const vector3& pt, int i)
{
	if (i >= mTetGenIO.numberofpoints || i < 0)
	{
		std::cout<<"index " << i << "out of range " << mTetGenIO.numberofpoints << "in TetMesh::SetIthVertex" << std::endl;
		return;
	}

	mTetGenIO.pointlist[3 * i] = pt.x;
	mTetGenIO.pointlist[3 * i + 1] = pt.y;
	mTetGenIO.pointlist[3 * i + 2] = pt.z;
}

int TetMesh::GetNumVertices() const
{
	return mTetGenIO.numberofpoints;
}

int TetMesh::GetNumTetrahedra() const
{
	return mTetGenIO.numberoftetrahedra;
}

Tetrahedron TetMesh::GetIthTetrahedron(int i) const
{
	if (i >= mTetGenIO.numberoftetrahedra || i < 0)
	{
		std::cout<<"index " << i << "out of range " << mTetGenIO.numberoftetrahedra << "in TetMesh::GetIthTetrahedron" << std::endl;
		assert(0);
	}

	assert(mTetGenIO.numberofcorners == 4);
	Tetrahedron tet;
	tet.mVertIndices[0] = mTetGenIO.tetrahedronlist[4 * i];
	tet.mVertIndices[1] = mTetGenIO.tetrahedronlist[4 * i + 1];
	tet.mVertIndices[2] = mTetGenIO.tetrahedronlist[4 * i + 2];
	tet.mVertIndices[3] = mTetGenIO.tetrahedronlist[4 * i + 3];
	tet.mVertices[0] = GetIthVertex(tet.mVertIndices[0]);
	tet.mVertices[1] = GetIthVertex(tet.mVertIndices[1]);
	tet.mVertices[2] = GetIthVertex(tet.mVertIndices[2]);
	tet.mVertices[3] = GetIthVertex(tet.mVertIndices[3]);
	return tet;

}

void TetMesh::SetRenderOption(SoftObjRenderOption option)
{
	mRenderOption = option;
}

void TetMesh::SetID(int id)
{
	mObjectID = id;
}

void TetMesh::SetPointColor(const vector<DecoColor>& colors)
{	
	mPointColors = colors;
}

void TetMesh::renderContactPolygon(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	mObj->RenderContactPolygon(RI);
}

void TetMesh::renderCenterOfMass(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
    DecoVertexBuffer vb;
    vector3 com(mCenterOfMass);
    vb.SetVertexInformation(1, &(com));
    RI->SetColor(DecoColor(0xff00ffff));
    RI->SetLineWidth(10);
    RI->DrawPrimitive(PT_PointList, 1, DT_SolidPolygon, &vb);
}

void TetMesh::renderCenterOfPressure(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	DecoVertexBuffer vb;
	vector3 cop(mCenterOfPressure);
	vb.SetVertexInformation(1, &(cop));
	RI->SetColor(DecoColor(0xff7fff7f));
	RI->SetLineWidth(10);
	RI->DrawPrimitive(PT_PointList, 1, DT_SolidPolygon, &vb);
}

void TetMesh::renderSelectedPoints(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	if (mSelectedPoints.empty())
		return;
	std::vector<vector3> allVertices;
	RetrieveAllVertices(allVertices);

	DecoVertexBuffer vb;
	vector<vector3> selectedPoints;
	int numSelectedPoints = static_cast<int>(mSelectedPoints.size());
	for (int i = 0; i < numSelectedPoints; ++i)
	{
		selectedPoints.push_back(allVertices[mSelectedPoints[i]]);
	}
	vb.SetVertexInformation(selectedPoints.size(), &(selectedPoints[0]));
	RI->SetColor(DecoColor(0xff0000ff));
	RI->SetLineWidth(8);
	RI->DrawPrimitive(PT_PointList, selectedPoints.size(), DT_SolidPolygon, &vb);
}
void TetMesh::renderSelected(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	renderSelectedPoints(RI, Lights, numLight, DrawType);
}

void TetMesh::renderPoints(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	std::vector<vector3> allVertices;
	RetrieveAllVertices(allVertices);
	DecoVertexBuffer vb;
	if (mPointColors.empty())
		vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	else
		vb.SetVertexInformation(allVertices.size(), &(allVertices[0]), NULL, NULL, &(mPointColors[0]));
	RI->SetColor(DecoColor(0xffffffff));
	RI->SetLineWidth(3);
	RI->DrawPrimitive(PT_PointList, allVertices.size(), DT_SolidPolygon, &vb);

}

void TetMesh::renderEdges(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	std::vector<vector3> allVertices;

	calculateVertices(allVertices);	
	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	RI->SetLineWidth(1);
	RI->SetColor(DecoColor(0xffffffff));
	RI->DrawPrimitive(PT_TriangleList, allVertices.size() / 3, DT_WireFrame, &vb);
}

void TetMesh::calculateVertices(std::vector<vector3>& vertices)
{
	vertices.resize(mTetGenIO.numberoftrifaces * 3);
	for (int i = 0; i < mTetGenIO.numberoftrifaces; ++i)
	{
		
		int indexA = mTetGenIO.trifacelist[3 * i];
		int indexB = mTetGenIO.trifacelist[3 * i + 1];
		int indexC = mTetGenIO.trifacelist[3 * i + 2];
		vector3 a(mTetGenIO.pointlist[3 * indexA], mTetGenIO.pointlist[3 * indexA + 1], mTetGenIO.pointlist[3 * indexA + 2]);
		vector3 b(mTetGenIO.pointlist[3 * indexB], mTetGenIO.pointlist[3 * indexB + 1], mTetGenIO.pointlist[3 * indexB + 2]);
		vector3 c(mTetGenIO.pointlist[3 * indexC], mTetGenIO.pointlist[3 * indexC + 1], mTetGenIO.pointlist[3 * indexC + 2]);
		vertices[3 * i] = a;
		vertices[3 * i + 1] = b;
		vertices[3 * i + 2] = c;
	}
}
void TetMesh::calculateNormals(std::vector<vector3>& normals)
{
	normals.resize(mTetGenIO.numberoftrifaces * 3);

	std::vector<vector3> pointNormals;
	std::vector<double> pointNormalWeights;
	pointNormals.resize(mTetGenIO.numberofpoints);
	pointNormalWeights.resize(mTetGenIO.numberofpoints);

	for (size_t i = 0; i < pointNormalWeights.size(); ++i)
	{
		pointNormalWeights[i] = 0;
	}

	for (int i = 0; i < mTetGenIO.numberoftrifaces; ++i)
	{
		int indexA = mTetGenIO.trifacelist[3 * i];
		int indexB = mTetGenIO.trifacelist[3 * i + 1];
		int indexC = mTetGenIO.trifacelist[3 * i + 2];
		vector3 a(mTetGenIO.pointlist[3 * indexA], mTetGenIO.pointlist[3 * indexA + 1], mTetGenIO.pointlist[3 * indexA + 2]);
		vector3 b(mTetGenIO.pointlist[3 * indexB], mTetGenIO.pointlist[3 * indexB + 1], mTetGenIO.pointlist[3 * indexB + 2]);
		vector3 c(mTetGenIO.pointlist[3 * indexC], mTetGenIO.pointlist[3 * indexC + 1], mTetGenIO.pointlist[3 * indexC + 2]);

		vector3 normal = -CrossProduct(b - a, c - a);
		double area = 0.5 * normal.length();
		normal.normalize();
		pointNormals[indexA] += area * normal;
		pointNormals[indexB] += area * normal;
		pointNormals[indexC] += area * normal;

		pointNormalWeights[indexA] += area;
		pointNormalWeights[indexB] += area;
		pointNormalWeights[indexC] += area;
		//		allNormals[3 * i] = allNormals[3 * i + 1] = allNormals[3 * i + 2] = normal;
	}
	for (int i = 0; i < mTetGenIO.numberoftrifaces; ++i)
	{
		int indexA = mTetGenIO.trifacelist[3 * i];
		int indexB = mTetGenIO.trifacelist[3 * i + 1];
		int indexC = mTetGenIO.trifacelist[3 * i + 2];
		normals[3 * i] = (pointNormals[indexA] / pointNormalWeights[indexA]).normalize();
		normals[3 * i + 1] = (pointNormals[indexB] / pointNormalWeights[indexB]).normalize();;
		normals[3 * i + 2] = (pointNormals[indexC] / pointNormalWeights[indexC]).normalize();;
	}
}

void TetMesh::renderMesh(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	mMaterial.Apply();
	for (int i = 0; i < numLight; ++i)
	{
		RI->SetLight(i, Lights[i]);
	}

	std::vector<vector3> allVertices;
	std::vector<vector3> allNormals;
	

	allVertices.resize(mTetGenIO.numberoftrifaces * 3);
	allNormals.resize(mTetGenIO.numberoftrifaces * 3);
	

	calculateVertices(allVertices);
	calculateNormals(allNormals);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]), &(allNormals[0]));
	RI->DrawPrimitive(PT_TriangleList, allVertices.size() / 3, DT_SolidPolygon, &vb);
	RI->ResetLight();

	bool bDebugNormal = false;
	if (bDebugNormal)
	{
		std::vector<vector3> normalVertices;
		for (size_t i = 0; i < allVertices.size(); ++i)
		{
			normalVertices.push_back(allVertices[i]);
			normalVertices.push_back(allVertices[i] + allNormals[i]);
		}
		DecoVertexBuffer vbNormal;
		vbNormal.SetVertexInformation(normalVertices.size(), &(normalVertices[0]));
		RI->DrawPrimitive(PT_LineList, normalVertices.size() / 2, DT_SolidPolygon, &vbNormal);

	}
}

void TetMesh::calculateIDColors(std::vector<DecoColor>& colors)
{
	colors.resize(mTetGenIO.numberoftrifaces * 3);
	for (int i = 0; i < mTetGenIO.numberoftrifaces; ++i)
	{
		BYTE lowBit = static_cast<BYTE>(i & 0x000000ff);
		BYTE highBit = static_cast<BYTE>((i >> 8) & 0x000000ff);
		colors[3 * i] = colors[3 * i + 1] = colors[3 * i + 2] = DecoColor(static_cast<BYTE>(mObjectID), highBit, lowBit);
	}
}

void TetMesh::renderTriangleID(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	RI->ResetLight();
	std::vector<vector3> allVertices;
	std::vector<DecoColor> allColors;

	allVertices.resize(mTetGenIO.numberoftrifaces * 3);
	allColors.resize(mTetGenIO.numberoftrifaces * 3);


	calculateVertices(allVertices);
	calculateIDColors(allColors);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]), NULL, NULL, &(allColors[0]));
	RI->DrawPrimitive(PT_TriangleList, allVertices.size() / 3, DT_SolidPolygon, &vb);
	
}

void TetMesh::calculateBarycentricColors(std::vector<DecoColor>& colors)
{
	colors.resize(mTetGenIO.numberoftrifaces * 3);
	for (int i = 0; i < mTetGenIO.numberoftrifaces; ++i)
	{
		colors[3 * i] = DecoColor(255, 0, 0);
		colors[3 * i + 1] = DecoColor(0, 255, 0);
		colors[3 * i + 2] = DecoColor(0, 0, 255);
	}
}


void TetMesh::renderBarycentricCoord(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{	
	RI->ResetLight();
	std::vector<vector3> allVertices;
	std::vector<DecoColor> allColors;

	allVertices.resize(mTetGenIO.numberoftrifaces * 3);
	allColors.resize(mTetGenIO.numberoftrifaces * 3);


	calculateVertices(allVertices);
	calculateBarycentricColors(allColors);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]), NULL, NULL, &(allColors[0]));
	RI->DrawPrimitive(PT_TriangleList, allVertices.size() / 3, DT_SolidPolygon, &vb);

}

void TetMesh::renderTetrahedra(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	for (int i = 0; i < numLight; ++i)
	{
		RI->SetLight(i, Lights[i]);
	}

	RI->ResetLight();
}


bool TetMesh::LoadFromFile(const string& fileName)
{
	bool ret = mTetGenIO.load_tetmesh(const_cast<char*>(fileName.c_str()));
	std::vector<vector3> allVertices;
	RetrieveAllVertices(allVertices);
	mbb = Bounding(&(allVertices[0]), allVertices.size());
	return ret;
}

void  TetMesh::WriteToFile(const string& fileName)
{
	mTetGenIO.save_nodes(const_cast<char*>(fileName.c_str()));
	mTetGenIO.save_elements(const_cast<char*>(fileName.c_str()));
	mTetGenIO.save_faces(const_cast<char*>(fileName.c_str()));
	mTetGenIO.save_poly(const_cast<char*>(fileName.c_str()));
}


Box TetMesh::GetBoundingBox()
{
	return GetUntransformedBoundingBox();
}
Box	TetMesh::GetUntransformedBoundingBox() const
{
	return mbb;
}
void TetMesh::SetLocalToWorld(const matrix44& matrix)
{

}
void TetMesh::RenderBoundingBox(DecoRenderInterface* RI)
{

}
BOOL TetMesh::RayIntersection(const vector3& origin, const vector3& dir, vector3& intersectionPt, DOUBLE& time, vector3* normal)
{
	return FALSE;
}
BOOL TetMesh::NeedSort()
{
	return FALSE;
}
void TetMesh::Render(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	RenderOpaqueSection(RI, Lights, numLight, DrawType);
}
void TetMesh::RenderTransparentSection(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{

}
void TetMesh::RenderOpaqueSection(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
    if (mRenderOption != RO_ID && mRenderOption != RO_Barycentric)
    {
		if (dynamic_cast<ActiveSoftBody*>(mObj) != NULL)
		{
			renderSelected(RI, Lights, numLight, DrawType);
			renderCenterOfMass(RI);
			renderCenterOfPressure(RI);
		}
		//renderContactPolygon(RI);
    }

	switch (mRenderOption)
	{
	case RO_Points:
		renderPoints(RI, Lights, numLight, DrawType);
		break;
	case RO_Edges:
		renderEdges(RI, Lights, numLight, DrawType);
		break;
	case RO_Mesh:
		renderMesh(RI, Lights, numLight, DrawType);
		break;
	case RO_Tetrahedra:
		renderTetrahedra(RI, Lights, numLight, DrawType);
		break;
	case RO_ID:
		renderTriangleID(RI, Lights, numLight, DrawType);
		break;
	case RO_Barycentric:
		renderBarycentricCoord(RI, Lights, numLight, DrawType);
		break;
	}
}
VOID TetMesh::Scale(DOUBLE xcoef, DOUBLE ycoef, DOUBLE zcoef)
{
	std::vector<vector3> allVertices;
	RetrieveAllVertices(allVertices);
	int numNodes = static_cast<int>(allVertices.size());
	for (int i = 0; i < numNodes; ++i)
	{
		allVertices[i].x *= xcoef;
		allVertices[i].y *= ycoef;
		allVertices[i].z *= zcoef;
		SetIthVertex(allVertices[i], i);
	}
}

int TetMesh::RayIntersectionCount(const vector3& origin, const vector3& dir)
{
	return 0;
}
BOOL TetMesh::IsPrimitiveInBox(Box& box)
{
	return FALSE;
}
void TetMesh::RetrieveAllVertices(vector<vector3>& allVertices)
{
	int numVertices = GetNumVertices();
	for (int i = 0; i < numVertices; ++i)
	{
		allVertices.push_back(GetIthVertex(i));
	}
}
BOOL TetMesh::IsPointInside(const vector3& pt, bool bVote)
{
	return FALSE;
}

void TetMesh::SetSelectPoint(const vector<int> indices)
{
	mSelectedPoints = indices;
}

void TetMesh::SetCenterOfMass(const Vector3d& com)
{
    mCenterOfMass = com;
}

void TetMesh::SetCenterOfPressure(const Vector3d& cop)
{
	mCenterOfPressure = cop;
}

void TetMesh::DumpToObjFile(const string& fileName)
{
    vector<int> indices;
    vector<int> newIndices;
    vector<vector3> vertices;
    vector<int> faceIndices;

    //newIndices.insert(newIndices.begin(), mTetGenIO.numberofpoints, -1);

    //for (int i = 0; i < mTetGenIO.numberoftrifaces; ++i)
    //{
    //    for (int j = 0; j < 3; ++j)
    //    {
    //        int indexA = mTetGenIO.trifacelist[3 * i + j];
    //        if (newIndices[indexA] == -1)
    //        {
    //            vector3 a(mTetGenIO.pointlist[3 * indexA], mTetGenIO.pointlist[3 * indexA + 1], mTetGenIO.pointlist[3 * indexA + 2]);
    //            newIndices[indexA] = static_cast<int>(vertices.size());
    //            vertices.push_back(a);
    //            indices.push_back(indexA);
    //        }
    //        faceIndices.push_back(newIndices[indexA]);
    //    }
    //}

	// Dump the tet mesh instead of the surface mesh.
	for (int i = 0; i < mTetGenIO.numberofpoints; ++i)
	{
		vector3 a(mTetGenIO.pointlist[3 * i], mTetGenIO.pointlist[3 * i + 1], mTetGenIO.pointlist[3 * i + 2]);
		vertices.push_back(a);
	}
	for (int i = 0; i < mTetGenIO.numberoftetrahedra; ++i)
	{
		int a = mTetGenIO.tetrahedronlist[4 * i];
		int b = mTetGenIO.tetrahedronlist[4 * i + 1];
		int c = mTetGenIO.tetrahedronlist[4 * i + 2];
		int d = mTetGenIO.tetrahedronlist[4 * i + 3];
		if (a == b || a == c || a == d || b == c || b == d || c == d)
			continue;

		if (a > mTetGenIO.numberofpoints || b > mTetGenIO.numberofpoints || c > mTetGenIO.numberofpoints || d > mTetGenIO.numberofpoints)
			LOG(INFO) << "Index out of range.";

		indices.clear();
		indices.push_back(a);
		indices.push_back(b);
		indices.push_back(c);
		indices.push_back(d);

		for (int j = 0; j < 4; ++j)
		{
			faceIndices.push_back(indices[(j + 0) % 4]);
			faceIndices.push_back(indices[(j + 1) % 4]);
			faceIndices.push_back(indices[(j + 2) % 4]);
		}
	}

	
	
    ofstream out(fileName.c_str());
    int numVertices = static_cast<int>(vertices.size());
    int numFaces = static_cast<int>(faceIndices.size()) / 3;

    for (int i = 0; i < numVertices; ++i)
        out<<"v "<< vertices[i].x <<" "<< vertices[i].y <<" "<< vertices[i].z << std::endl;

    for (int i = 0; i < numFaces; ++i)
        out<<"f "<< faceIndices[3 * i + 0] + 1 <<" "<< faceIndices[3 * i + 1] + 1 <<" "<< faceIndices[3 * i + 2] + 1 << std::endl;


}
