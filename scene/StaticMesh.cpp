#include "StaticMesh.h"
#include "render/ase.h"
#include "render/light.h"
#include "math.h"
#include "utility/DecoLogger.h"
#include <fstream>
using namespace std;

DecoStaticMeshSection::~DecoStaticMeshSection()
{
	Clear();
}

DecoStaticMeshSection::DecoStaticMeshSection(const DecoStaticMeshSection& rhs)
: m_vb(NULL), m_ib(NULL), m_pre_u_tiling(1.0f), m_pre_v_tiling(1.0f),
m_numPrimitives(0),  m_primType(PT_TriangleList)
{
	CopyFrom(rhs);
}

DecoStaticMeshSection& DecoStaticMeshSection::operator =(const DecoStaticMeshSection& rhs)
{	
	if (&rhs == this) return *this;
	Clear();
	CopyFrom(rhs);
	return *this;
}

void DecoStaticMeshSection::Render(DecoRenderInterface* RI, DecoDrawType DrawType)
{
	BOOL bHasMaterial = (m_vb->GetMaterialRef() >= 0);
	BOOL bIsTransparent = FALSE;
	if (bHasMaterial)
	{		
		RI->SetMaterial(*(m_pStaticMesh->m_materials[m_vb->GetMaterialRef()]));		
		bIsTransparent = m_pStaticMesh->m_materials[m_vb->GetMaterialRef()]->IsTransparent();
	}
	RI->PushState(RS_RenderType);
	if (bIsTransparent)
		RI->BeginRenderType(RT_Transparent);
	else
		RI->BeginRenderType(RT_Opaque);
	RI->DrawPrimitive(m_primType, m_numPrimitives, DrawType, m_vb);
	RI->EndRenderType();
	RI->PopState(RS_RenderType);
}

BOOL DecoStaticMeshSection::NeedSort ()
{
	if (m_vb->GetMaterialRef() < 0) return FALSE;
	return m_pStaticMesh->m_materials[m_vb->GetMaterialRef()]->IsTransparent();
}

VOID DecoStaticMeshSection::Init(const ASE::GeomObject& obj)
{
	m_vb = new DecoVertexBuffer();
	m_vb->Init(obj.mesh, m_boundingBox);
	AdjustTexCoords();
	m_numPrimitives = obj.mesh.face_count; 
}



BOOL DecoStaticMeshSection::ReadFromFile(ifstream& inFile)
{
	Clear();
	inFile.read((char *)&m_numPrimitives, sizeof(size_t));
	inFile.read((char *)&m_primType, sizeof(DecoPrimitiveType));

	m_vb = new DecoVertexBuffer();
	m_vb->ReadFromFile(inFile, m_boundingBox);

	// handling transparent
	//if (m_pStaticMesh->GetMaterial(m_vb->GetMaterialRef())->IsTransparent())
	//{
	//	m_vb->SetColorBufferForTransparent(m_pStaticMesh->GetMaterial(m_vb->GetMaterialRef()));
	//}

	return TRUE;
}

VOID DecoStaticMeshSection::WriteToFile(ofstream& outFile)
{
	outFile.write((char *)&m_numPrimitives, sizeof(size_t));
	outFile.write((char *)&m_primType, sizeof(DecoPrimitiveType));
	m_vb->WriteToFile(outFile);

}

// adjust texture coords according to material's attributes such as UV tiling and UV offsets. 
// this function should be called after m_pMaterials has been set properly.
// TODO: handle more texture coords manipulations here
VOID DecoStaticMeshSection::AdjustTexCoords()
{	
	if (m_vb->GetTexcoords() == NULL)
		return ; // no texture coords to be adjust

	assert(m_pStaticMesh);

	// ONLY Consider MAP_DIFFUSE Here. 
	// DecoMaterial* pMat = &(m_pStaticMesh->m_materials[m_vb.GetMaterialRef()]);

	if (m_pStaticMesh->m_materials.size() == 0)
		return ;

	if (m_vb->GetMaterialRef() == -1)
		return;

	vector<DecoMaterial*>::const_iterator pMat = m_pStaticMesh->m_materials.begin() + m_vb->GetMaterialRef();
	const DecoTexture* pTexture = (*pMat)->GetFirstMapDiffuse();
	if (pTexture == NULL)
		return ;

	vector3* pTexCoord = m_vb->GetTexcoords();
	size_t   num = m_vb->NumberVertices();

	FLOAT uTile = pTexture->GetUTiling() / m_pre_u_tiling;
	FLOAT vTile = pTexture->GetVTiling() / m_pre_v_tiling;
	m_pre_u_tiling = pTexture->GetUTiling();
	m_pre_v_tiling = pTexture->GetVTiling();

	FLOAT uOffset = pTexture->GetUOffset();
	FLOAT vOffset = pTexture->GetVOffset();

	for (size_t i = 0; i < num; ++i)
	{
		pTexCoord[i].x *= uTile;
		pTexCoord[i].x += uOffset;
		pTexCoord[i].y *= vTile;
		pTexCoord[i].y += vOffset;
	}

	//ROTATE
	FLOAT angle = pTexture->GetUVWAngle();	
	if (angle > 0.001)
	{
		FLOAT cos_value = cos(angle);
		FLOAT sin_value = sin(angle);
		DOUBLE x = 0.0f, y = 0.0f;
		for (size_t i = 0; i < num; ++i)
		{			
			x = pTexCoord[i].x;
			y = pTexCoord[i].y;

			pTexCoord[i].x = x * cos_value - y * sin_value;
			pTexCoord[i].y = x * sin_value + y * cos_value;
		}
	}
}

BOOL DecoStaticMeshSection::RayIntersection(const vector3& origin, const vector3& dir, vector3& intersectionPt, DOUBLE& time, vector3* normal)
{
	assert(m_primType == PT_TriangleList);
	assert(m_vb->NumberVertices() == m_numPrimitives * 3);

	intersectionPt = origin;
	DOUBLE t = -1;
	BOOL bIntersect = FALSE;
	vector3 storedA, storedB, storedC;
	vector3 storedNormA, storedNormB, storedNormC;
	for (size_t ithTriangle = 0; ithTriangle < m_numPrimitives; ithTriangle++)
	{
		vector3 vertex1 = m_vb->Vertices(ithTriangle * 3);
		vector3 vertex2 = m_vb->Vertices(ithTriangle * 3 + 1);
		vector3 vertex3 = m_vb->Vertices(ithTriangle * 3 + 2);		
		DOUBLE intersectT = 0;

		if(TriangleRayIntersection (vertex1, vertex2, vertex3, origin, dir, &intersectionPt, &intersectT))
		{
			if (t < 0 || t > intersectT)
			{
				t = intersectT;
				storedA = vertex1;
				storedB = vertex2;
				storedC = vertex3;
				if (normal && m_vb->ParseFieldFlag(USE_NORMAL_STREAM))
				{
					storedNormA = m_vb->Normals(ithTriangle * 3);
					storedNormB = m_vb->Normals(ithTriangle * 3 + 1);
					storedNormC = m_vb->Normals(ithTriangle * 3 + 2);
				}
				bIntersect = TRUE;
			}
		}
	}
	if (bIntersect)
	{
		time = t;
		intersectionPt = (origin + t * dir);
		if (normal && m_vb->ParseFieldFlag(USE_NORMAL_STREAM))
		{
			vector3 coeff;
			matrix33 A;
			A[0] = storedA;
			A[1] = storedB;
			A[2] = storedC;
			coeff = A.invert() * intersectionPt;
			normal->x = coeff.x * storedNormA.x + coeff.y * storedNormB.x + coeff.z * storedNormC.x;
			normal->y = coeff.x * storedNormA.y + coeff.y * storedNormB.y + coeff.z * storedNormC.y;
			normal->z = coeff.x * storedNormA.z + coeff.y * storedNormB.z + coeff.z * storedNormC.z;
		}
	}
	return bIntersect;
}

int DecoStaticMeshSection::RayIntersectionCount(const vector3& origin, const vector3& dir)
{
	assert(m_primType == PT_TriangleList);
	assert(m_vb->NumberVertices() == m_numPrimitives * 3);
	std::vector<vector3> intersectPoints;
	int count = 0;
	vector3 intersectionPt = origin;
	double intersectT = 0;
	for (size_t ithTriangle = 0; ithTriangle < m_numPrimitives; ithTriangle++)
	{
		vector3 vertex1 = m_vb->Vertices(ithTriangle * 3);
		vector3 vertex2 = m_vb->Vertices(ithTriangle * 3 + 1);
		vector3 vertex3 = m_vb->Vertices(ithTriangle * 3 + 2);		
		DOUBLE intersectT = 0;

		if(TriangleRayIntersection (vertex1, vertex2, vertex3, origin, dir, &intersectionPt, &intersectT))
		{
			if(intersectT > -EPSILON_FLOAT)		
			{
				bool discard = false;
				for (std::vector<vector3>::iterator it = intersectPoints.begin(); it != intersectPoints.end(); ++it)
				{
					if ((intersectionPt - (*it)).lengthSqr() < EPSILON_FLOAT)
					{
						discard = true;
						break;
					}
				}
				if (!discard)
				{
					intersectPoints.push_back(intersectionPt);
					++count;
				}
			}
		}
	}
	return count;
}

BOOL DecoStaticMeshSection::IsPrimitiveInBox(const Box& box)
{
	assert(m_primType == PT_TriangleList);
	assert(m_vb->NumberVertices() == m_numPrimitives * 3);
	for (size_t ithTriangle = 0; ithTriangle < m_numPrimitives; ithTriangle++)
	{
		vector3 vertex1 = m_vb->Vertices(ithTriangle * 3);
		vector3 vertex2 = m_vb->Vertices(ithTriangle * 3 + 1);
		vector3 vertex3 = m_vb->Vertices(ithTriangle * 3 + 2);	
		if (box.TriangleInBox(vertex1, vertex2, vertex3))
			return TRUE;
	}
	return FALSE;
}

void DecoStaticMeshSection::Scale(DOUBLE coef)
{
	vector3 *p_vertex = m_vb->GetVertices();
	matrix44 scaleMatrix = ScaleMatrix44(coef, coef, coef);

	for (size_t i = 0; i < m_vb->NumberVertices(); ++i)
	{
		vector4 homoPoint(p_vertex[i].x, p_vertex[i].y, p_vertex[i].z, 1.f);
		vector4 transPoint = scaleMatrix * homoPoint;
		p_vertex[i] = vector3(transPoint.x, transPoint.y, transPoint.z);
	}
	m_boundingBox = Bounding(p_vertex, m_vb->NumberVertices());
}

void DecoStaticMeshSection::Scale(DOUBLE xcoef, DOUBLE ycoef, DOUBLE zcoef)
{
	vector3 *p_vertex = m_vb->GetVertices();
	matrix44 scaleMatrix = ScaleMatrix44(xcoef, ycoef, zcoef);

	for (size_t i = 0; i < m_vb->NumberVertices(); ++i)
	{
		vector4 homoPoint(p_vertex[i].x, p_vertex[i].y, p_vertex[i].z, 1.f);
		vector4 transPoint = scaleMatrix * homoPoint;
		p_vertex[i] = vector3(transPoint.x, transPoint.y, transPoint.z);
	}
	m_boundingBox = Bounding(p_vertex, m_vb->NumberVertices());
}

VOID DecoStaticMeshSection::Clear()
{
	if (m_vb)
	{
		delete m_vb;
		m_vb = NULL;
	}

	if (m_ib)
	{
		delete m_ib;
		m_ib = NULL;
	}
}

VOID DecoStaticMeshSection::CopyFrom(const DecoStaticMeshSection& rhs)
{
	Clear();

	m_primType = rhs.m_primType;
	m_numPrimitives = rhs.m_numPrimitives;
	//m_pStaticMesh = rhs.m_pStaticMesh;

	m_pre_u_tiling = rhs.m_pre_u_tiling;
	m_pre_v_tiling = rhs.m_pre_v_tiling;

	m_boundingBox = rhs.m_boundingBox;

	if (rhs.m_vb)
		m_vb = new DecoVertexBuffer(*(rhs.m_vb));
	if (rhs.m_ib)
		m_ib = new DecoIndexBuffer(*(rhs.m_ib));
}

// This only adjust Diffuse Map's UTiling and VTiling
VOID DecoStaticMeshSection::SetTextureRepeat(FLOAT uRepeat, FLOAT vRepeat)
{
	vector<DecoMaterial*>::iterator pMat = m_pStaticMesh->m_materials.begin() + m_vb->GetMaterialRef();
	DecoTexture* pTexture = const_cast<DecoTexture*>((*pMat)->GetFirstMapDiffuse());
	if (NULL == pTexture) return;
	assert(pTexture);
	pTexture->SetUTiling(uRepeat);
	pTexture->SetVTiling(vRepeat);
	AdjustTexCoords();
}

VOID DecoStaticMeshSection::ExchangeYZAxis()
{
	m_vb->ExchangeYZAxis();
	m_boundingBox = BoundingBox();
}

VOID DecoStaticMeshSection::CenterModel(vector3 center)
{	
	m_vb->CenterModel(center);
	m_boundingBox = Bounding(m_vb->GetVertices(), m_vb->NumberVertices());
}

// DecoStaticMesh's Constructor , do nothing
DecoStaticMesh::DecoStaticMesh() : m_num_materials(0), m_num_sections(0)
{
	m_localToWorld.identity();
}

DecoStaticMesh::DecoStaticMesh(const DecoStaticMesh& rhs): m_num_materials(0), m_num_sections(0)
{
	Clear();
	CopyFrom(rhs);
}

DecoStaticMesh::~DecoStaticMesh()
{
	Clear();
}

void DecoStaticMeshSection::RetrieveAllVertices(vector<vector3>& allVertices)
{
	vector3* posList = m_vb->GetVertices();
	assert(m_primType == PT_TriangleList);
	for (UINT i = 0; i < m_numPrimitives; i++)
	{
		allVertices.push_back(posList[3 * i]);
		allVertices.push_back(posList[3 * i + 1]);
		allVertices.push_back(posList[3 * i + 2]);
	}
}

void DecoStaticMeshSection::RetrieveAllNormals(vector<vector3>& allNormals)
{
	vector3* normList = m_vb->GetNormals();
	if (!normList) return;
	assert(m_primType == PT_TriangleList);
	for (UINT i = 0; i < m_numPrimitives; i++)
	{
		allNormals.push_back(normList[3 * i]);
		allNormals.push_back(normList[3 * i + 1]);
		allNormals.push_back(normList[3 * i + 2]);
	}
}

DecoStaticMesh& DecoStaticMesh::operator =(const DecoStaticMesh& rhs)
{
	if (&rhs == this) return *this;
	Clear();
	CopyFrom(rhs);
	return *this;
}

// implemented by bovine
VOID DecoStaticMesh::ReadFromFile(const string& fileName)
{
	ASE::ASEFile ase_file;
	ase_file.read(fileName.c_str());

	// TODO: CHECK ASE FILE STATE HERE

	// Create Materials 
	for (vector<ASE::Material>::const_iterator iter_material = ase_file.materials.begin();
		iter_material != ase_file.materials.end(); ++iter_material)
	{
		DecoMaterial	*material = new DecoMaterial;

		material->Init(*iter_material);

		if (material->IsValid())
			m_materials.push_back(material);
		else
			assert(0); // TODO: HANDLE ERROR HERE;
	}
	assert (m_materials.size() == ase_file.materials.size());
	m_num_materials = m_materials.size();

	// Create Geometry Objects
	for (vector<ASE::GeomObject>::iterator iter_obj = ase_file.geomobjects.begin(); 
		iter_obj != ase_file.geomobjects.end(); ++iter_obj)
	{
		DecoStaticMeshSection* pSec = new DecoStaticMeshSection(this);
		pSec->Init(*iter_obj);
		m_sections.push_back(pSec);
	}
	assert (m_sections.size() == ase_file.geomobjects.size());
	m_num_sections = m_sections.size();
	CalculateBoundingBox();
}

void DecoStaticMesh::readObj(const char* fname, vector<vector3>& vertices, vector<Index3INT>& faces)
{
	ifstream in(fname, ios::in);
	char c[20];
	int vt = 0;
	int vn = 0;
	int i;

	while ( (in >> c) && (strcmp(c,"v") != 0) );

	if ( strcmp(c,"v") == 0 ) {
		i=0;
		while ( strcmp(c,"v") == 0 ) {
			vector3 vertex;
			in >> vertex.x;
			in >> vertex.y;
			in >> vertex.z;
			vertices.push_back(vertex);
			in >> c;
			i++;
		}
	}

	while ( (strcmp(c,"f") != 0) && (strcmp(c,"s") != 0) && (strcmp(c,"vt") != 0) && (strcmp(c,"vn") != 0) && ( in >> c) );

	if ( strcmp(c,"vt") == 0 ) {
		vt = 1;
		do {
			double temp;
			in >> temp;
			in >> temp;
			in >> c;
		} while ( strcmp(c,"vt") == 0 );
	}

	while ( (strcmp(c,"f") != 0) && (strcmp(c,"s") != 0)  && (strcmp(c,"vn") != 0) && ( in >> c) );

	if ( strcmp(c,"vn") == 0 ) {
		vn = 1;
		while ( strcmp(c,"vn") == 0 ) {
			double temp;
			in >> temp;
			in >> temp;
			in >> temp;
			in >> c;
		}
	}

	while ( (strcmp(c,"f") != 0 && strcmp(c, "s") != 0) && ( in >> c) );
	m_smoothGroupInfos.clear();
	i=0;
	do 
	{
		if (strcmp(c, "s") == 0)
		{
			int groupId = 0;
			in >> groupId;
			SmoothGroupInfo sgInfo;
			sgInfo.mGroupId = groupId;
			sgInfo.mStartFaceIdx = i;
			m_smoothGroupInfos.push_back(sgInfo);
		}
		if ( strcmp(c,"f") == 0 ) 
		{
			int temp;
			char ct;
			Index3INT index;
			in >> index.m_i;
			if ( vt ) { in >> ct; in >> temp; }
			if ( vn ) { in >> ct; in >> temp; }

			in >> index.m_j;
			if ( vt ) { in >> ct; in >> temp; }
			if ( vn ) { in >> ct; in >> temp; }

			in >> index.m_k;
			if ( vt ) { in >> ct; in >> temp; }
			if ( vn ) { in >> ct; in >> temp; }
			index.m_i -= 1;
			index.m_j -= 1;
			index.m_k -= 1;
			faces.push_back(index);
			i++;
		}
	}	while ( in >> c );

}

void readCheckFile(char* filename, vector<BOOL>& checkList)
{
	ifstream in(filename, ios::in);
	char c[20];

	while ( (in >> c) )
	{
		if (!strcmp(c,"("))
		{
			for (INT i = 0; i < 3; i++)
			{
				in >> c;
				INT index = atoi(c);
				if (index == -1)
					continue;
				checkList[index - 1] = TRUE;
			}
		}
		else if (c[0] == '(')
		{
			INT index = atoi(&c[1]);
			checkList[index - 1] = TRUE;
			for (INT i = 0; i < 2; i++)
			{
				in >> c;
				INT index = atoi(c);
				if (index == -1)
					continue;
				checkList[index - 1] = TRUE;
			}
		}
	}
}

void DecoStaticMesh::computeColor(const vector<vector3>& vertices, const vector<Index3INT>& faces, vector<DecoColor>& cols)
{
		INT numVertics = static_cast<INT>(vertices.size());
		vector<vector3> faceNormals;
		vector<BOOL> checkFace;
		checkFace.resize(numVertics);
		cols.resize(numVertics);
		readCheckFile("intersect.txt", checkFace);
		for (INT i=0; i<numVertics; i++)
		{
			if (checkFace[i])
			{
				cols[i] = DecoColor(0xffff0000);
			}
			else
			{
				cols[i] = DecoColor(0xff00ff00);
			}
		}
}

void DecoStaticMesh::computeNormals(const vector<vector3>& vertices, const vector<Index3INT>& faces, vector<vector3>& norms)
{
	INT numFaces = static_cast<INT>(faces.size());
	INT numVertics = static_cast<INT>(vertices.size());
	vector<vector3> faceNormals;
	vector<BOOL> checkFace;
	checkFace.resize(numVertics);
	faceNormals.resize(numFaces);
	norms.resize(numVertics);
	//readCheckFile("intersect.txt", checkFace);
	for (INT i = 0; i < numVertics; i++)
	{
		norms[i] = vector3(0, 0, 0);
		checkFace[i] = FALSE;
	}
	for (INT i=0; i<numFaces; i++)
	{
		vector3 a = (vertices[faces[i].m_j]-vertices[faces[i].m_i]).normalize();
		vector3 b = (vertices[faces[i].m_k]-vertices[faces[i].m_i]).normalize();
		faceNormals[i] = CrossProduct(a, b);
		faceNormals[i].normalize();
		norms[faces[i].m_i]+=faceNormals[i];
		norms[faces[i].m_j]+=faceNormals[i];
		norms[faces[i].m_k]+=faceNormals[i];
	}

	for (INT i=0; i<numVertics; i++)
	{
		if (checkFace[i])
		{
			norms[i] = vector3(0, 0, 0);
		}
		else
		{
			norms[i].normalize();
		}
	}
}

void DecoStaticMesh::DumpToObjFile(const string& fileName)
{
	ofstream out(fileName.c_str());
	std::vector<vector3> allVertices;
	RetrieveAllVertices(allVertices);
	int numVertices = static_cast<int>(allVertices.size());
	int numFaces = numVertices / 3;
	for (int i = 0; i < numVertices; i++)
		out<<"v "<< allVertices[i].x <<" "<< allVertices[i].y <<" "<< allVertices[i].z << std::endl;
	if (m_smoothGroupInfos.size())
	{
		int numSmoothGroups = static_cast<int>(m_smoothGroupInfos.size());
		for (int i = 0; i < numSmoothGroups - 1; ++i)
		{
			out << "s " << m_smoothGroupInfos[i].mGroupId << std::endl;
			for (int j = m_smoothGroupInfos[i].mStartFaceIdx; j < m_smoothGroupInfos[i + 1].mStartFaceIdx; ++j)
			{
				out<<"f "<< j * 3 + 1 <<" "<< j * 3 + 2<<" "<< j * 3 + 3 << std::endl;
			}

		}
		out << "s " << m_smoothGroupInfos[numSmoothGroups - 1].mGroupId << std::endl;
		for (int j = m_smoothGroupInfos[numSmoothGroups - 1].mStartFaceIdx; j < numFaces; ++j)
		{
			out<<"f "<< j * 3 + 1 <<" "<< j * 3 + 2<<" "<< j * 3 + 3 << std::endl;
		}
	}
	else
	{
		for (int i = 0; i < numFaces; i++)
			out<<"f "<< i * 3 + 1 <<" "<< i * 3 + 2<<" "<< i * 3 + 3 << std::endl;
	}
}

VOID DecoStaticMesh::ReadFromObjFile(const string& fileName)
{
	vector<vector3> vertices;
	vector<Index3INT> faces;
	vector<vector3> norms;
	vector<DecoColor> cols;
	readObj(fileName.c_str(), vertices, faces);
	computeNormals(vertices, faces, norms);
	//computeColor(vertices, faces, cols);

	INT numFaces = static_cast<INT>(faces.size());
	vector3* verts = new vector3[3 * numFaces];
	//DecoColor* colors = new DecoColor[3 * numFaces];
	vector3* vertNorm = new vector3[3 * numFaces];
	DecoVertexBuffer* vb = new DecoVertexBuffer();
	for (INT i = 0; i < numFaces; i++)
	{
		verts[3 * i] = vertices[faces[i].m_i];
		verts[3 * i + 1] = vertices[faces[i].m_j];
		verts[3 * i + 2] = vertices[faces[i].m_k];
		//colors[3 * i] = cols[faces[i].m_i];
		//colors[3 * i + 1] = cols[faces[i].m_j];
		//colors[3 * i + 2] = cols[faces[i].m_k];
		vertNorm[3 * i] = norms[faces[i].m_i];
		vertNorm[3 * i + 1] = norms[faces[i].m_j];
		vertNorm[3 * i + 2] = norms[faces[i].m_k];
	}
	vb->SetVertexInformation(3 * numFaces, verts, vertNorm);
	//vb->SetVertexInformation(3 * numFaces, verts, NULL, NULL, colors);
	this->CreateFromVertexBuffer(vb);
}

void DecoStaticMesh::Render(DecoRenderInterface* RI, DecoLight** Lights, INT numLights, DecoDrawType DrawType)
{
	RI->PushState(RS_ModelMatrix);
	if (DrawType == DT_SolidPolygon)
	{
		for (INT ithLight = 0; ithLight < numLights; ithLight++)
		{
			assert(Lights && Lights[ithLight]);
			RI->SetLight(ithLight, Lights[ithLight]);
		}
	}
	RI->SetTransform(TT_LocalToWorld, m_localToWorld);

	for (vector<DecoStaticMeshSection*>::iterator sectionIter = m_sections.begin(); sectionIter != m_sections.end(); sectionIter++)
	{
		if ((*sectionIter)->NeedSort()) continue;
		(*sectionIter)->Render(RI, DrawType);
	}
	for (vector<DecoStaticMeshSection*>::iterator sectionIter = m_sections.begin(); sectionIter != m_sections.end(); sectionIter++)
	{
		if ((*sectionIter)->NeedSort())
			(*sectionIter)->Render(RI, DrawType);
	}
	RI->ResetLight();
	RI->PopState(RS_ModelMatrix);
}

void DecoStaticMesh::RenderOpaqueSection(DecoRenderInterface* RI, DecoLight** Lights, INT numLights, DecoDrawType DrawType)
{
	RI->PushState(RS_ModelMatrix);
	if (DrawType == DT_SolidPolygon)
	{
		for (INT ithLight = 0; ithLight < numLights; ithLight++)
		{
			assert(Lights && Lights[ithLight]);
			RI->SetLight(ithLight, Lights[ithLight]);
		}
	}
	RI->SetTransform(TT_LocalToWorld, m_localToWorld);

	for (vector<DecoStaticMeshSection*>::iterator sectionIter = m_sections.begin(); sectionIter != m_sections.end(); sectionIter++)
	{
		if ((*sectionIter)->NeedSort()) continue;
		(*sectionIter)->Render(RI, DrawType);
	}
	RI->ResetLight();
	RI->PopState(RS_ModelMatrix);
}

void DecoStaticMesh::RenderTransparentSection(DecoRenderInterface* RI, DecoLight** Lights, INT numLights, DecoDrawType DrawType)
{
	{
		RI->PushState(RS_ModelMatrix);
		if (DrawType == DT_SolidPolygon)
		{
			for (INT ithLight = 0; ithLight < numLights; ithLight++)
			{
				assert(Lights && Lights[ithLight]);
				RI->SetLight(ithLight, Lights[ithLight]);
			}
		}
		RI->SetTransform(TT_LocalToWorld, m_localToWorld);

		for (vector<DecoStaticMeshSection*>::iterator sectionIter = m_sections.begin(); sectionIter != m_sections.end(); sectionIter++)
		{
			if ((*sectionIter)->NeedSort()) 
				(*sectionIter)->Render(RI, DrawType);
		}
		RI->ResetLight();
		RI->PopState(RS_ModelMatrix);
	}
}

void DecoStaticMesh::RenderBoundingBox(DecoRenderInterface* RI)
{
}
Box DecoStaticMesh::GetBoundingBox()
{
   	return m_boundingBox.TransformBy(m_localToWorld);
}

Box DecoStaticMesh::GetUntransformedBoundingBox() const
{
  	return m_boundingBox;
}
void DecoStaticMesh::SetLocalToWorld(const matrix44& localToWorld)
{
 	m_localToWorld = localToWorld;
}

matrix44 DecoStaticMesh::GetLocalToWorld() const
{
	return m_localToWorld;
}

VOID DecoStaticMesh::CalculateBoundingBox()
      {
	vector3 maxPoint(MIN_DOUBLE, MIN_DOUBLE, MIN_DOUBLE);
	vector3 minPoint(MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE);
	m_boundingBox.Set(minPoint, maxPoint);
 	for (vector<DecoStaticMeshSection*>::iterator sectionIter = m_sections.begin(); sectionIter != m_sections.end(); sectionIter++)
	{
		Box box = (*sectionIter)->BoundingBox();
		if (box[0].x < minPoint.x)
			minPoint.x = box[0].x;
		if (box[1].x > maxPoint.x)
			maxPoint.x = box[1].x;
		if (box[0].y < minPoint.y)
			minPoint.y = box[0].y;
		if (box[1].y > maxPoint.y)
			maxPoint.y = box[1].y;
		if (box[0].z < minPoint.z)
			minPoint.z = box[0].z;
		if (box[1].z > maxPoint.z)
			maxPoint.z = box[1].z;
	}
	m_boundingBox.Set(minPoint, maxPoint);
}

BOOL DecoStaticMesh::IsPointInside(const vector3& pt, bool bUseVote)
{
	const Box& bb = GetBoundingBox();
	if (checkWithInBox(pt, bb.GetMin(), bb.GetMax()))
	{
		if (!bUseVote)
		{
			int intersectionCount = RayIntersectionCount(pt, vector3(1,0,0));
			if (intersectionCount % 2)
			{
				return TRUE;
			}
			else
			{
				return FALSE;
			}
		}
		else
		{
			int numVotesInside = 0;
			int intersectionCount1 = RayIntersectionCount(pt, vector3(1,0,0));
			int intersectionCount2 = RayIntersectionCount(pt, vector3(0,1,0));
			if (intersectionCount1 % 2 && intersectionCount2 % 2)
				return TRUE;
			else if (intersectionCount1 % 2 == 0 && intersectionCount2 % 2 == 0)
				return FALSE;

			else
			{
				int intersectionCount3 = RayIntersectionCount(pt, vector3(0,0,1));
				if (intersectionCount3 % 2)
					return TRUE;
				else
					return FALSE;
			}
		}
	}
	else
		return FALSE;
}



void DecoStaticMesh::Scale(DOUBLE coef)
{
	for (vector<DecoStaticMeshSection*>::iterator iter_sec = m_sections.begin();
		iter_sec != m_sections.end(); ++iter_sec)
	{
		(*iter_sec)->Scale(coef);
	}
	CalculateBoundingBox();
}

void DecoStaticMesh::Scale(DOUBLE xcoef, DOUBLE ycoef, DOUBLE zcoef)
{
	for (vector<DecoStaticMeshSection*>::iterator iter_sec = m_sections.begin();
		iter_sec != m_sections.end(); ++iter_sec)
	{
		(*iter_sec)->Scale(xcoef, ycoef, zcoef);
	}
	CalculateBoundingBox();
}

void DecoStaticMesh::ScaleTo(DOUBLE xcoef, DOUBLE ycoef, DOUBLE zcoef)
{
	vector3 bbSize = m_boundingBox.GetExtent();
	for (vector<DecoStaticMeshSection*>::iterator iter_sec = m_sections.begin();
		iter_sec != m_sections.end(); ++iter_sec)
	{
		(*iter_sec)->Scale(xcoef / bbSize.x, ycoef / bbSize.y, zcoef / bbSize.z);
	}
	CalculateBoundingBox();
}

//void DecoStaticMesh::CreateTestInstance()
//{
//	DecoStaticMeshSection testSection(this);
//	DecoVertexBuffer testVB;
//
//	vector3 InVertices[] = {vector3(-1.f, -1.f, -1.f), vector3(1.f, -1.f, -1.f), vector3(1.f, -1.f, 1.f), vector3(-1.f, -1.f, 1.f), 
//							vector3(-1.f, 1.f, -1.f), vector3(1.f, 1.f, -1.f), vector3(1.f, 1.f, 1.f), vector3(-1.f, 1.f, 1.f) };
//	DecoColor InColors[] = {DecoColor(0xffff0000), DecoColor(0xff00ffff), DecoColor(0xff00ff00), DecoColor(0xff0000ff), 
//							DecoColor(0xffffff00), DecoColor(0xffff00ff), DecoColor(0xffffffff), DecoColor(0xff000fff)};
//	//vector3 InVertices[] = {vector3(1.f, 1.f, 0.f), vector3(0.f, 1.f, 0.f), vector3(0.f, 0.f, 0.f) };
//	//DecoColor InColors[] = {DecoColor(0xffffffff), DecoColor(0xffffffff), DecoColor(0xffffffff)};
//
//	testVB.SetVertexInformation(8, InVertices);
//	//testVB.FillColorInformation(InColors);
//	
//
//	SHORT InIndices[] = {0, 3, 1, 3, 2, 1,
//						 7, 4, 5, 7, 5, 6, 
//						 4, 3, 0, 4, 7, 3, 
//						 2, 5, 1, 2, 6, 5, 
//						 4, 0, 5, 5, 0, 1, 
//						 3, 7, 6, 3, 6, 2};
//	DecoIndexBuffer testIB;
//	testIB.SetIndexInformation(InIndices, 36);
//	testSection.SetVertexBuffer(testVB);
//	testSection.SetIndexBuffer(testIB);
//	testSection.SetNumPrimitives(12);
//	m_sections.push_back(testSection);	
//}

VOID DecoStaticMesh::Clear()
{
	for (vector<DecoMaterial*>::iterator i = m_materials.begin(); i != m_materials.end(); ++i)
	{
		if (*i)
		{
			delete *i;
			*i = NULL;
		}
	}
	m_materials.clear();

	for (vector<DecoStaticMeshSection*>::iterator i = m_sections.begin(); i != m_sections.end(); ++i)
	{
		if (*i)
		{
			delete *i;
			*i = NULL;
		}
	}
	m_sections.clear();

	m_localToWorld.identity();
}

VOID DecoStaticMesh::CopyFrom(const DecoStaticMesh& rhs)
{
	Clear();
	for (vector<DecoMaterial*>::const_iterator i = rhs.m_materials.begin(); i != rhs.m_materials.end(); ++i)
	{
		DecoMaterial* pMat = new DecoMaterial(**i);
		m_materials.push_back(pMat);
	}
	for (vector<DecoStaticMeshSection*>::const_iterator i = rhs.m_sections.begin(); i != rhs.m_sections.end(); ++i)
	{
		DecoStaticMeshSection *pSec = new DecoStaticMeshSection(**i);
		pSec->m_pStaticMesh = this;
		m_sections.push_back(pSec);
	}
	m_num_sections = rhs.m_num_sections;
	m_num_materials = rhs.m_num_materials;


	m_boundingBox = rhs.m_boundingBox;
	m_localToWorld = rhs.m_localToWorld;
}

BOOL DecoStaticMesh::ReadFromFile(ifstream& inFile)
{
	Clear();

	inFile.read((char *)&m_num_materials, sizeof(size_t));
	for (size_t i = 0; i < m_num_materials; ++i)
	{
		DecoMaterial *pMat = new DecoMaterial();
		pMat->ReadFromFile(inFile);
		m_materials.push_back(pMat);
	}

	inFile.read((char *)&m_num_sections, sizeof(size_t));
	for (size_t i = 0; i < m_num_sections; ++i)
	{
		DecoStaticMeshSection *pSec = new DecoStaticMeshSection(this);
		pSec->ReadFromFile(inFile);
		m_sections.push_back(pSec);
	}

	CalculateBoundingBox();
	
	return TRUE;
}

VOID DecoStaticMesh::WriteToFile(ofstream& outFile)
{	
	outFile.write((char *)&m_num_materials, sizeof(size_t));
	for (size_t i = 0; i < m_num_materials; ++i)
		m_materials[i]->WriteToFile(outFile);

	outFile.write((char *)&m_num_sections, sizeof(size_t));
	for (size_t i = 0; i < m_num_sections; ++i)
		m_sections[i]->WriteToFile(outFile);
}

size_t DecoStaticMesh::AddMaterial(DecoMaterial* material)
{
	m_materials.push_back(material);
	++m_num_materials;
	assert(m_num_materials == m_materials.size());
	return m_num_materials - 1;
}

size_t DecoStaticMesh::AddStaticMeshSection(DecoStaticMeshSection* section)
{
	m_sections.push_back(section);
	++m_num_sections;
	assert(m_num_sections == m_sections.size());
	return m_num_sections - 1;
}

DecoMaterial* DecoStaticMesh::GetMaterial(size_t index)
{
	assert(index < m_num_materials);
	return m_materials[index];
}

DecoMaterial* DecoStaticMesh::GetFirstValidMaterial()
{
	for (INT i = 0; i < static_cast<INT>(m_num_sections); i++)
	{
		DecoStaticMeshSection* section = m_sections[i];
		if (section->GetMaterialRef() >= 0)
			return m_materials[section->GetMaterialRef()];
	}
	return NULL;
}
DecoStaticMeshSection* DecoStaticMesh::GetStaticMeshSection(size_t index)
{
	assert(index < m_num_sections);
	return m_sections[index];
}

VOID DecoStaticMesh::SetTextureRepeat(FLOAT uRepeat, FLOAT vRepeat)
{
	for (size_t i = 0; i < m_num_sections; ++i)
	{
		m_sections[i]->SetTextureRepeat(uRepeat, vRepeat);
	}
}

BOOL DecoStaticMesh::RayIntersection(const vector3& origin, const vector3& dir, vector3& intersectionPt, DOUBLE& time, vector3* normal)
{
	matrix44 invLocalToWorld = m_localToWorld;
	invLocalToWorld.invert();
	vector4 homoOrigin(origin);
	homoOrigin.w = 1;
	vector4 tempOrigin = invLocalToWorld * homoOrigin;
	vector3 localOrigin(tempOrigin.x, tempOrigin.y, tempOrigin.z);
	vector4 tempDir(invLocalToWorld * dir);
	vector3 localDir(tempDir.x, tempDir.y, tempDir.z);
	localDir.normalize();
	DOUBLE t = -1;
	DOUBLE intersectT = 0;
	vector3 normCandidate;

	for (vector<DecoStaticMeshSection*>::const_iterator ithSection = m_sections.begin(); ithSection != m_sections.end(); ++ithSection)
	{
		if ((*ithSection)->RayIntersection(localOrigin, localDir, intersectionPt, intersectT, &normCandidate))
		{
			if (t < 0 || t > intersectT)
			{
				t = intersectT;
				if (normal)
					*normal = normCandidate;
			}
		}
	}
	if (t > 0)
	{
		vector3 localIntersection = localOrigin + t * localDir;
		intersectionPt = TransformPoint(m_localToWorld, localIntersection);
		if (normal)
			*normal = TransformNormal(m_localToWorld, *normal);

		time = t;
	}
	return (t > 0);	
}

VOID DecoStaticMesh::ExchangeYZAxis()
{
	for (vector<DecoStaticMeshSection*>::iterator ithSection = m_sections.begin(); ithSection != m_sections.end(); ++ithSection)
	{
		(*ithSection)->ExchangeYZAxis();
	}
}

VOID DecoStaticMesh::CenterModel()
{
	this->CalculateBoundingBox();
	vector3 center = m_boundingBox.GetCenter();
	for (vector<DecoStaticMeshSection*>::iterator ithSection = m_sections.begin(); ithSection != m_sections.end(); ++ithSection)
	{
		(*ithSection)->CenterModel(center);
	}
	this->CalculateBoundingBox();
}

BOOL DecoStaticMesh::NeedSort()
{
	for (vector<DecoMaterial*>::iterator it = m_materials.begin(); it != m_materials.end(); it++)
	{
		if ((*it)->IsTransparent())
			return TRUE;
	}
	return FALSE;
}

int DecoStaticMesh::RayIntersectionCount(const vector3& origin, const vector3& dir)
{
	matrix44 invLocalToWorld = m_localToWorld;
	invLocalToWorld.invert();
	vector4 homoOrigin(origin);
	homoOrigin.w = 1;
	vector4 tempOrigin = invLocalToWorld * homoOrigin;
	vector3 localOrigin(tempOrigin.x, tempOrigin.y, tempOrigin.z);
	vector4 tempDir(invLocalToWorld * dir);
	vector3 localDir(tempDir.x, tempDir.y, tempDir.z);
	localDir.normalize();

	int count = 0;
	for (vector<DecoStaticMeshSection*>::const_iterator ithSection = m_sections.begin(); ithSection != m_sections.end(); ++ithSection)
	{
		count += (*ithSection)->RayIntersectionCount(localOrigin, localDir);
		/*if ((*ithSection)->RayIntersection(localOrigin, localDir, intersectionPt, intersectT))
		{
			if (t < 0 || t > intersectT)
			{
				t = intersectT;
			}
		}*/
	}
	return count;
}

BOOL DecoStaticMesh::IsPrimitiveInBox(Box& box)
{
	matrix44 invLocalToWorld = m_localToWorld;
	invLocalToWorld.invert();
	vector4 homoMin(box[0].x, box[0].y, box[0].z, 1);
	vector4 homoMax(box[1].x, box[1].y, box[1].z, 1);
	vector4 tmpMin = invLocalToWorld * homoMin;
	vector4 tmpMax = invLocalToWorld * homoMax;
	vector3 localMin(homoMin.x, homoMin.y, homoMin.z);
	vector3 localMax(homoMax.x, homoMax.y, homoMax.z);
	const Box localBox(localMin, localMax);
	for (vector<DecoStaticMeshSection*>::const_iterator ithSection = m_sections.begin(); ithSection != m_sections.end(); ++ithSection)
	{
		if ((*ithSection)->IsPrimitiveInBox(localBox))
			return TRUE;
	}
	return FALSE;
}
void DecoStaticMesh::RetrieveAllVertices(vector<vector3>& allVertices)
{
	allVertices.clear();
	for (vector<DecoStaticMeshSection*>::const_iterator i = m_sections.begin(); i != m_sections.end(); ++i)
	{
		(*i)->RetrieveAllVertices(allVertices);
	}
	for (size_t i = 0; i < allVertices.size(); i++)
	{
		vector4 homoVertex(allVertices[i].x, allVertices[i].y, allVertices[i].z, 1);
		vector4 transformedVertex = m_localToWorld * homoVertex;
		allVertices[i] = vector3(transformedVertex.x, transformedVertex.y, transformedVertex.z);
	}
}

void DecoStaticMesh::RetrieveAllNormals(vector<vector3>& allNormals)
{
	allNormals.clear();
	vector<vector3> allVert;
	for (vector<DecoStaticMeshSection*>::const_iterator i = m_sections.begin(); i != m_sections.end(); ++i)
	{
		(*i)->RetrieveAllNormals(allNormals);
	}
	for (size_t i = 0; i < allNormals.size(); i++)
	{
		allNormals[i] = TransformNormal(m_localToWorld, allNormals[i]);
		//vector4 homoNormal(allNormals[i].x, allNormals[i].y, allNormals[i].z, 0);
		//matrix44 normalTransformMatrix = m_localToWorld.invert().transpose();
		//vector4 transformedVertex = normalTransformMatrix * homoNormal;
		//allNormals[i] = vector3(transformedVertex.x, transformedVertex.y, transformedVertex.z);
	}
}

void DecoStaticMesh::CreateFromVertexBuffer(DecoVertexBuffer** vb, INT numVB)//assume vb[i] get materialRef i
{
	for (INT i = 0; i < numVB; i++)
	{
		DecoStaticMeshSection* testSection = new DecoStaticMeshSection(this);
		vb[i]->SetMaterialRef(i);

		testSection->SetVertexBuffer(vb[i]);
		vector3 *p_vertex = vb[i]->GetVertices();
		testSection->m_boundingBox = Bounding(p_vertex, vb[i]->NumberVertices());
		testSection->m_pStaticMesh = this;
		assert(vb[i]->NumberVertices() % 3 == 0);
		testSection->SetPrimitiveType(PT_TriangleList);
		testSection->SetNumPrimitives(vb[i]->NumberVertices() / 3);

		m_sections.push_back(testSection);	
		++m_num_sections;
		CalculateBoundingBox();
	}

}

void DecoStaticMesh::CreateFromVertexBuffer(DecoVertexBuffer* vb)
{
	DecoStaticMeshSection* testSection = NULL;
	if (!m_sections.size())
	{
		testSection = new DecoStaticMeshSection(this);
		if (vb->GetMaterialRef() < 0)
		{
			vb->SetMaterialRef(0);
			DecoMaterial *mat = new DecoMaterial(); //something to do here
			mat->SetDiffuse(vector4(0.8, 0.5, 0.5, 1));
			AddMaterial(mat);
		}
	}
	else
		testSection = m_sections[0];

	testSection->SetVertexBuffer(vb);
	vector3 *p_vertex = vb->GetVertices();
	testSection->m_boundingBox = Bounding(p_vertex, vb->NumberVertices());
	vector3 center = testSection->m_boundingBox.GetCenter();
	vb->CenterModel(center);
	testSection->m_boundingBox = Bounding(p_vertex, vb->NumberVertices());
	testSection->m_pStaticMesh = this;
	assert(vb->NumberVertices() % 3 == 0);
	testSection->SetPrimitiveType(PT_TriangleList);
	testSection->SetNumPrimitives(vb->NumberVertices() / 3);
	if (!m_sections.size())
		m_sections.push_back(testSection);	
	++m_num_sections;
	CalculateBoundingBox();
}

DOUBLE DecoStaticMesh::PointDistance(const vector3& pt)
{
	vector<vector3> vertices;
	RetrieveAllVertices(vertices);
	DOUBLE minDist = MAX_DOUBLE;
	size_t numVertices = vertices.size();
	assert (numVertices % 3 == 0);
	for (UINT ithVertex = 0; ithVertex < numVertices; ithVertex += 3)
	{
		vector3& a = vertices[ithVertex];
		vector3& b = vertices[ithVertex + 1];
		vector3& c = vertices[ithVertex + 2];
		minDist = min(minDist, PointTriangleDistance(pt, a, b, c));
	}
	assert(minDist < MAX_DOUBLE);
	return minDist;
}

//void DecoStaticMesh::GetAbVextexInformation(AbVecList& vertexList, AbTriList& faceList, AbVecList& normList)
//{
//	vector<vector3> vertices;
//	vector<vector3> normals;
//	vector<Index3INT> faces;
//	faces.clear();
//	RetrieveAllVertices(vertices);
//	RetrieveAllNormals(normals);
//	INT vertexSize = static_cast<INT>(vertices.size());
//	INT normalSize = static_cast<INT>(normals.size());
//	vertexList.setNumVecs(static_cast<INT>(vertexSize));
//	normList.setNumVecs(static_cast<INT>(vertexSize));
//	faceList.setNumTriangles(vertexSize / 3);
//	assert(0 == vertexSize % 3 && vertexSize == normals.size());
//	for (INT i = 0; i < vertexSize / 3; i++)
//	{
//		vertexList[3 * i] = SmVector3(vertices[3 * i].x, vertices[3 * i].y, vertices[3 * i].z);
//		faceList[i].a = 3 * i;
//		
//		vertexList[3 * i + 1] = SmVector3(vertices[3 * i + 1].x, vertices[3 * i + 1].y, vertices[3 * i + 1].z);
//		faceList[i].b = 3 * i + 1;
//
//		vertexList[3 * i + 2] = SmVector3(vertices[3 * i + 2].x, vertices[3 * i + 2].y, vertices[3 * i + 2].z);
//		faceList[i].c = 3 * i + 2;	
//
//		faces.push_back(Index3INT(3 * i, 3 * i + 1, 3 * i + 2));
//		//vector3 a = CrossProduct((vertices[3 * i + 1] - vertices[3 * i]).normalize(), (vertices[3 * i + 2] - vertices[3 * i]).normalize()).normalize();
//		//vector3 b = CrossProduct((vertices[3 * i + 2] - vertices[3 * i + 1]).normalize(), (vertices[3 * i] - vertices[3 * i + 1]).normalize()).normalize();
//		//vector3 c = CrossProduct((vertices[3 * i] - vertices[3 * i + 2]).normalize(), (vertices[3 * i + 1] - vertices[3 * i + 2]).normalize()).normalize();
//
//		//normList[3 * i] = SmVector3(a.x, a.y, a.z);
//		//normList[3 * i + 1] = SmVector3(b.x, b.y, b.z);
//		//normList[3 * i + 2] = SmVector3(c.x, c.y, c.z);
//	}  
//	if (!normals.size())
//		computeNormals(vertices, faces, normals);
//	for (INT i = 0; i < vertexSize / 3; i++)
//	{
//		normList[3 * i] = SmVector3(normals[3 * i].x, normals[3 * i].y, normals[3 * i].z);
//		normList[3 * i + 1] = SmVector3(normals[3 * i + 1].x, normals[3 * i + 1].y, normals[3 * i + 1].z);
//		normList[3 * i + 2] = SmVector3(normals[3 * i + 2].x, normals[3 * i + 2].y, normals[3 * i + 2].z);
//	}
//	
//
//}

void DecoStaticMesh::DumpRibFile(const string& filename, BOOL bAppend)
{
	//ofstream out(filename.c_str(), bAppend ? ios::app : ios::out);
	//UINT i;
	//AbVecList vertexList;
	//AbTriList faceList;
	//AbVecList normList;
	//GetAbVextexInformation(vertexList, faceList, normList);

	//UINT numTriangles = faceList.numTriangles();
	//UINT numVertices = vertexList.numVecs();
	//UINT numNormals = normList.numVecs();

	//out<<"PointsPolygons [";
	//for (i=0; i<numTriangles; i++)
	//	out<<"3 ";
	//out<<"] [";
	//for (i=0; i<numTriangles; i++)
	//	out<<faceList[i].a<<" "<<faceList[i].b<<" "<<faceList[i].c<<" ";
	//out<<"]\n";

	//out<<"\"P\" [";
	//for (i=0; i < numVertices; i++)
	//	out<<vertexList[i][0]<<" "<<vertexList[i][1]<<" "<<vertexList[i][2]<<" ";
	//out<<"]\n\"N\" [";
	//for (i=0; i < numNormals; i++)
	//	out<<normList[i][0]<<" "<<normList[i][1]<<" "<<normList[i][2]<<" ";
	//out<<"]\n\n";
}
