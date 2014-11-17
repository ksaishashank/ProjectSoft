#include "MuscleFiber.h"
#include "TetMesh.h"
#include "ActiveSoftBody.h"
#include "MuscleDof.h"

MuscleFiber::MuscleFiber() : mSoftBody(NULL), mInfluenceSigma(0.5)
{

}

MuscleFiber::MuscleFiber(const vector<Vector3d>& vertices)
{
	SetRestShapePosition(vertices);
	SetPosition(vertices);
}	
MuscleFiber::~MuscleFiber()
{

}

void MuscleFiber::SetSoftBody(ActiveSoftBody* softBody)
{
	mSoftBody = softBody;
}

const vector<Vector3d>& MuscleFiber::GetPositions() const
{
	return mVertices;
}

void MuscleFiber::SetRestShapePosition(const vector<Vector3d>& pos)
{
	mVerticesRestShape = pos;
}

void MuscleFiber::SetPosition(const vector<Vector3d>& pos)
{
	mVertices = pos;
}
void MuscleFiber::SetTetIdx(const vector<int>& indices)
{
	mTetIndex = indices;

}
void MuscleFiber::SetBarycentricCoord(const vector<Vector3d>& coord)
{
	mBaryCentricCoordInTet = coord;
}

Matrix3d MuscleFiber::GetRotationFromXAxis(int segmentId)
{
	Vector3d fiberSegmentDir = mVerticesRestShape[segmentId + 1] - mVerticesRestShape[segmentId];
	fiberSegmentDir.normalize();
	Vector3d xDir;
	xDir << 1.0, 0.0, 0.0;
	Vector3d axis = xDir.cross(fiberSegmentDir);
	if (axis.dot(axis) > 1e-6)
	{
		axis.normalize();
		double angle = acos(xDir.dot(fiberSegmentDir));
		AngleAxis<double> rot(angle, axis);
		Matrix3d rotMat = rot.toRotationMatrix();
		return rotMat;
	}
	else
	{
		return Matrix3d::Identity();
	}
}

Matrix3d MuscleFiber::Contract(int segmentId, double magnitude)
{
	Matrix3d xDirStress = Matrix3d::Zero();
	xDirStress(0, 0) = magnitude;
	Matrix3d rotMat = GetRotationFromXAxis(segmentId);
	Matrix3d stress = rotMat * xDirStress * rotMat.transpose();
	return stress;

}

void MuscleFiber::UpdatePosition()
{
	int numTetIdx = static_cast<int>(mTetIndex.size());
	for (int i = 0; i < numTetIdx; ++i)
	{
		TetMesh* tetMesh = mSoftBody->GetDeformedShapeTet();
		Tetrahedron tet = tetMesh->GetIthTetrahedron(mTetIndex[i]);
		vector3 newPos = mBaryCentricCoordInTet[i](0) * tet.mVertices[0]
					   + mBaryCentricCoordInTet[i](1) * tet.mVertices[1] 
					   + mBaryCentricCoordInTet[i](2) * tet.mVertices[2]  
					   + (1.0 - mBaryCentricCoordInTet[i].sum()) * tet.mVertices[3];
		mVertices[i] = Vector3d(newPos.x, newPos.y, newPos.z);
	}
}

void MuscleFiber::UpdateBarycentricCoordAndTetIndex()
{
	int numVertices = static_cast<int>(mVertices.size());
	mBaryCentricCoordInTet.resize(numVertices);
	mTetIndex.resize(numVertices);

	TetMesh* tetMesh = mSoftBody->GetRestShapeTet();
	int numTets = tetMesh->GetNumTetrahedra();
	for (int ithVert = 0; ithVert < numVertices; ++ithVert)
	{
		bool bFound = false;
		for (int ithTet = 0; ithTet < numTets; ++ithTet)
		{
			Tetrahedron tet = tetMesh->GetIthTetrahedron(ithTet);
			vector3 pt(mVerticesRestShape[ithVert](0), mVerticesRestShape[ithVert](1), mVerticesRestShape[ithVert](2));
			if (tet.PointInside(pt))
			{
				vector3 coord = tet.BaryCentricCoord(pt);
				mBaryCentricCoordInTet[ithVert] << coord.x, coord.y, coord.z;
				mTetIndex[ithVert] = ithTet;
				bFound = true;
			}
			if (bFound)
				break;
		}
		CHECK(bFound) << "The " << ithVert << " vertex in the muscle fiber " << mVerticesRestShape[ithVert] << " is not associated with a tet."; 
	}
}

const vector<Vector3d>& MuscleFiber::GetPolylineRepresentation() const
{
	return mVertices;
}

const vector<Vector3d>& MuscleFiber::GetPolylineRepresentationRestShape() const
{
	return mVerticesRestShape;
}


int MuscleFiber::GetNumSegments() const
{
	return static_cast<int>(mVertices.size() - 1);
}

double MuscleFiber::GetCurrentLength(int ithSegment) const
{
	Vector3d vec = mVertices[ithSegment + 1] - mVertices[ithSegment];
	return vec.norm();
}
double MuscleFiber::GetRestLength(int ithSegment) const
{
	Vector3d vec = mVerticesRestShape[ithSegment + 1] - mVerticesRestShape[ithSegment];
	return vec.norm();
}

void MuscleFiber::Translate(const Vector3d& offset)
{
	int numVertices = static_cast<int>(mVertices.size());
	for (int i = 0; i < numVertices; ++i)
	{
		mVertices[i] += offset;
		mVerticesRestShape[i] += offset;
	}
}

double MuscleFiber::GetInfluenceSigma() const
{
	return mInfluenceSigma;
}
void MuscleFiber::SetInfluenceSigma(double sigma)
{
	mInfluenceSigma = sigma;
}

void MuscleFiber::ConstructTestLongitudinalFiber(ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, int ithFiber)
{
	SetSoftBody(softBody);
	vector<Vector3d> fiberVertices;
	mInfluenceSigma = 0.1;
	int numVertices = 13;
	for (int i = 0; i < numVertices; ++i)
	{
		Vector3d pt = Vector3d::Zero();
		pt[0] = -10 + i * 20.0 / (numVertices - 1);
		fiberVertices.push_back(pt);

	}
	const int numDof = 6;
	int numSegments = numVertices - 1;
	int numSegmentsPerDof = numSegments / numDof;
	for (int i = 0; i < numDof; ++i)
	{
		MuscleDof dof;
		dof.SetSoftBody(mSoftBody);
		dof.BindWithMuscleFiber(ithFiber, i * numSegmentsPerDof, numSegmentsPerDof);
		muscleDofs.push_back(dof);
	}
	//MuscleDof dof;
	//dof.SetSoftBody(mSoftBody);
	//dof.BindWithMuscleFiber(ithFiber, 0, numVertices - 1);
	//muscleDofs.push_back(dof);


	SetPosition(fiberVertices);
	SetRestShapePosition(fiberVertices);
}

void MuscleFiber::AddDof(MuscleDof* dof)
{
	mDofs.push_back(dof);
}

void MuscleFiber::SetCurrentAsRest()
{
    UpdatePosition();	
    int numVertices = static_cast<int>(mVertices.size());
    for (int i = 0; i < numVertices; ++i)
    {
        mVerticesRestShape[i] = mVertices[i];
    }

}

void MuscleFiber::ConstructTestTwistFiber(ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, int ithFiber)
{
	SetSoftBody(softBody);
	vector<Vector3d> fiberVertices;
	mInfluenceSigma = 0.1;
	int numVertices = 100;
	int numSegsPerCircle = 10;
	double thetaStep = 2 * PI / numSegsPerCircle;
	double heightStep = 20.0 / (numVertices - 1);
	for (int i = 0; i < numVertices; ++i)
	{
		{
			Vector3d pt = Vector3d::Zero();
			pt[0] = -10 + i * heightStep;
			pt[1] = 0.8 * cos(i * thetaStep);
			pt[2] = 1.5 * sin(i * thetaStep);
			fiberVertices.push_back(pt);
		}
	}
	const int numDof = 1;
	int numSegments = numVertices - 1;
	int numSegmentsPerDof = numSegments / numDof;
	for (int i = 0; i < numDof; ++i)
	{
		MuscleDof dof;
		dof.SetSoftBody(mSoftBody);
		dof.BindWithMuscleFiber(ithFiber, i * numSegmentsPerDof, numSegmentsPerDof);
		muscleDofs.push_back(dof);
	}
	SetPosition(fiberVertices);
	SetRestShapePosition(fiberVertices);
}

void MuscleFiber::ConstructTestRadialVerticalFiber(ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, int ithFiber)
{
	SetSoftBody(softBody);
	vector<Vector3d> fiberVertices;
	mInfluenceSigma = 3.0;
	fiberVertices.push_back(Vector3d(0, -0.9, 0));
	fiberVertices.push_back(Vector3d(0, 0.9, 0));

	MuscleDof dof;
	dof.SetSoftBody(mSoftBody);
	dof.BindWithMuscleFiber(ithFiber, 0, 1);
	muscleDofs.push_back(dof);

	SetPosition(fiberVertices);
	SetRestShapePosition(fiberVertices);
}

void MuscleFiber::ConstructTestRadialHorizontalFiber(ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, int ithFiber)
{
	SetSoftBody(softBody);
	vector<Vector3d> fiberVertices;
	mInfluenceSigma = 3.0;
	fiberVertices.push_back(Vector3d(0.0, 0.0, -1.8));
	fiberVertices.push_back(Vector3d(0.0, 0.0, 1.8));

	MuscleDof dof;
	dof.SetSoftBody(mSoftBody);
	dof.BindWithMuscleFiber(ithFiber, 0, 1);
	muscleDofs.push_back(dof);

	SetPosition(fiberVertices);
	SetRestShapePosition(fiberVertices);
}

int MuscleFiber::ReadFromFile(ifstream& in, ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, vector<MuscleDof>& specialDofs, int ithFiber)
{
    SetSoftBody(softBody);
    CHECK(mSoftBody);
    int groupIndex = 0, numVertices, numDofs = 1, specialDofId = -1;
    char firstLine[256];
    in.getline(firstLine, 256);
    if (string(firstLine) == "")
        return -1; //end of file reached
    istringstream line(firstLine);
    line >> skipws >> numVertices >> numDofs >> groupIndex >> specialDofId;

    vector<Vector3d> fiberVertices;
    for (int i = 0; i < numVertices; ++i)
    {
        char moreLine[256];
        in.getline(moreLine, 256);
        istringstream vertPos(moreLine);
        int fiberId;
        float x, y, z;
        vertPos >> skipws >> fiberId >> x >> y >> z;
        fiberVertices.push_back(Vector3d(x, y, z));
    }
    int numSegmentsPerDof = (numVertices - 1) / numDofs;
    if (specialDofId == -1)
    {
        for (int i = 0; i < numDofs; ++i)
        {
            MuscleDof dof;
            dof.SetSoftBody(mSoftBody);
            dof.BindWithMuscleFiber(ithFiber, i * numSegmentsPerDof, numSegmentsPerDof);
            muscleDofs.push_back(dof);
        }
    }
    else
    {

        if (specialDofs.size() <= specialDofId)
        {
            specialDofs.resize(specialDofId + 1);

            MuscleDof dof;
            dof.SetSoftBody(mSoftBody);
            dof.BindWithMuscleFiber(ithFiber, 0, numVertices - 1);

            specialDofs[specialDofId] = dof;
        }
        else
        {
            specialDofs[specialDofId].BindWithMuscleFiber(ithFiber, 0, numVertices - 1);
        }
    }

    SetPosition(fiberVertices);
    SetRestShapePosition(fiberVertices);
    return groupIndex;
}

DecoArchive& operator<< (DecoArchive& Ar, const MuscleFiber& fiber)
{
	Ar << fiber.mVertices;
	Ar << fiber.mVerticesRestShape;
	Ar << fiber.mBaryCentricCoordInTet;
	Ar << fiber.mTetIndex;
	Ar << fiber.mInfluenceSigma;
	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, MuscleFiber& fiber)
{
	Ar >> fiber.mVertices;
	Ar >> fiber.mVerticesRestShape;
	Ar >> fiber.mBaryCentricCoordInTet;
	Ar >> fiber.mTetIndex;
	Ar >> fiber.mInfluenceSigma;
	return Ar;
}

