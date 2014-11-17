#include "DecoSoftBody.h"
#include "collision/CollisionDetector.h"
#include "ForceRecorder.h"
#include "PlayBackFrame.h"

DecoSoftObject::DecoSoftObject() : //the unit is dm and kg
DecoSceneObject(SoftT),
mMass(0),
mRho(0.1),
mYoungsModulus(1e5),
mPoissonRatio(0.45),
mDampingConstant1(0),
mDampingConstant2(1.0),//1.0
mFrictionCoeff(0.2),
mSolver(NULL),
mRenderOption(RO_Mesh),
mIsFrictionAnisotropic(false),
mbNeedRecalculateForce(true),
mTimeStep(0.0333),
mGravityConstant(98)
{
	mCOP = mCOPLast = Vector3d::Zero();
	bIsRenderable = bIsSelectable = bIsRotatable = bIsScalable = bIsMovable = TRUE;
	mSolver = new ODESolver(ST_ImplicitEuler, this);
	bIsUpdatable = TRUE;
	objID = gsObjId;
	mRestShape = new TetMesh();
	mDeformedShape = new TetMesh();
	mRestShape->SetObject(this);
	mRestShape->SetID(objID);
	mDeformedShape->SetObject(this);
	mDeformedShape->SetID(objID);
	mForces = new ForceRecorder();
	mForces->SetRecordingObject(this);
	gsObjId++;
}

DecoSoftObject::DecoSoftObject(const DecoSoftObject& rhs)
{
	copyFrom(rhs);
}

DecoSoftObject& DecoSoftObject::operator= (const DecoSoftObject& rhs)
{
	if (this == &rhs)
		return *this;
	copyFrom(rhs);
	return *this;
}


DecoSoftObject::~DecoSoftObject()
{
	if (mSolver)
		delete mSolver;
	if (mDeformedShape)
		delete mDeformedShape;
	if (mRestShape)
		delete mRestShape;
	if (mForces)
		delete mForces;
	ClearAllConstraints();
}

void DecoSoftObject::SetDensity(double density)
{
	mRho = density;
	computeMass();
}

void DecoSoftObject::SetDampingCoeff(double damp1, double damp2)
{
    mDampingConstant1 = damp1;
    mDampingConstant2 = damp2;
    precomputation();
}

double DecoSoftObject::GetYoungsModulus() const
{
	return mYoungsModulus;
}

void DecoSoftObject::SetYoungsModulus(double modulus)
{
	mYoungsModulus = modulus;
	precomputation();
}

void DecoSoftObject::SetPoissonRatio(double poissonRatio)
{
	mPoissonRatio = poissonRatio;
	precomputation();
}

void DecoSoftObject::SetFrictionCoeff(double friction, bool isAnisotropic)
{
	mFrictionCoeff = friction;
	mIsFrictionAnisotropic = isAnisotropic;
}

bool DecoSoftObject::IsFrictionAnisotropic() const
{
	return mIsFrictionAnisotropic;
}

void DecoSoftObject::copyFrom(const DecoSoftObject& rhs)
{
	*mRestShape = *rhs.mRestShape;
	*mDeformedShape = *rhs.mDeformedShape;

	mNodes = rhs.mNodes;
	mRestShapeInvertedX = rhs.mRestShapeInvertedX;
	mFrictionCoeff = rhs.mFrictionCoeff;
	mRho = rhs.mRho;
	mPoissonRatio = rhs.mPoissonRatio;
	mYoungsModulus = rhs.mYoungsModulus;
	mDampingConstant1 = rhs.mDampingConstant1;
	mDampingConstant2 = rhs.mDampingConstant2;
	mbNeedRecalculateForce = rhs.mbNeedRecalculateForce;

	mMassMatrix = rhs.mMassMatrix;
	
	*mSolver = *rhs.mSolver;
//	mFixVertexIndices = rhs.mFixVertexIndices;
	mRenderOption = rhs.mRenderOption;
	
	ClearAllConstraints();
	int numConstraints = static_cast<int>(rhs.mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		SoftBodyVertexConstraint* constraint = new SoftBodyVertexConstraint(*(rhs.mConstraints[i]));
		mConstraints.push_back(constraint);
	}

}

BOOL DecoSoftObject::CreateSceneObjectFromMesh(const std::string & path, double timeStep)
{
	mTimeStep = timeStep;
	BOOL bSuccess1 = mRestShape->LoadFromFile(path);
	BOOL bSuccess2 = mDeformedShape->LoadFromFile(path);
//	Rotate(AXIS_Z, PI / 2);
	name = path;
    if (strstr(name.c_str(), "ground") != NULL)
    {	
        mRho = 1e10;
    }
	if (bSuccess1 && bSuccess2)
	{
//		mDeformedShape.Scale(1.0, 1.0, 0.0);
		int numNode = mRestShape->GetNumVertices();
		mForces->SetNumNodes(numNode);

		for (int i = 0; i < numNode; ++i)
		{
			DecoSoftObjectNode node;
			node.mPos = mDeformedShape->GetIthVertex(i);
			node.mVel = vector3(0, 0, 0);
			node.mForceAccumulator = vector3(0, 0, 0);
			node.mMass = 0;
			mNodes.push_back(node);
		}
		precomputation();
		//constructTestConstraint();
		return TRUE;

	}
	return FALSE;
}

Vector3d DecoSoftObject::GetTorque()
{
    vector3 com = GetCenterOfMass();
    int numNodes = static_cast<int>(mNodes.size());
    vector3 torque(0, 0, 0);
    for (int i = 0; i < numNodes; ++i)
    {
        vector3 r = mNodes[i].mPos - com;
        vector3 f = mNodes[i].mForceAccumulator;
        torque += CrossProduct(r, f);
    }
    return Vector3d(torque.x, torque.y, torque.z);
}

Vector3d DecoSoftObject::GetCenterOfPressure() const
{
	return mCOP;
}

Vector3d DecoSoftObject::GetCenterOfPressureLast() const
{
	return mCOPLast;
}

Vector3d DecoSoftObject::GetVelOfCOM() const
{
	VectorXd vel;
	GetVel(vel);
	Vector3d comV = mCenterOfMassMulitplier * vel;
	return comV;
}

Vector3d DecoSoftObject::GetVelOfCOP() const
{
	return (mCOP - mCOPLast) / mTimeStep;
}

double DecoSoftObject::GetGravityConstant() const
{
    return mGravityConstant;
}
void DecoSoftObject::SetGravityConstant(double gravity)
{
    mGravityConstant = gravity;
}

void DecoSoftObject::UpdateCenterOfPressure(const set<int>& contactTriangleIndices, const set<int>& contactNodeIndices)
{
	double totalPressure = 0;
	vector3 cop(0, 0, 0);
    //////// method 1
	//for (set<int>::const_iterator it = contactTriangleIndices.begin(); it != contactTriangleIndices.end(); ++it)
	//{
	//	int ithTriangle = *it;
	//	int indexA, indexB, indexC;
	//	mDeformedShape->GetVertexIndicesForIthTriangle(ithTriangle, indexA, indexB, indexC);

	//	vector3 vertexA = mDeformedShape->GetIthVertex(indexA);
	//	vector3 vertexB = mDeformedShape->GetIthVertex(indexB);
	//	vector3 vertexC = mDeformedShape->GetIthVertex(indexC);

	//	vector3 contactForce = mForces->mNormalContactForces[indexA] + mForces->mNormalContactForces[indexB] + mForces->mNormalContactForces[indexC];
	//	//contactForce += mForces->mFrictionContactForces[indexA] + mForces->mFrictionContactForces[indexB] + mForces->mFrictionContactForces[indexC];
	//	contactForce /= 3;
	//	double pressure = contactForce.length() / abs(TriangleArea(vertexA, vertexB, vertexC));
	//	vector3 center = (vertexA + vertexB + vertexC) / 3.0;
	//	totalPressure += pressure;
	//	cop += pressure * center;
	//}
	//cop /= totalPressure;

    //////// method 2
	//for (vector<int>::const_iterator it = contactNodeIndices.begin(); it != contactNodeIndices.end(); ++it)
	//{
	//	int indexA = *it;

	//	vector3 vertexA = mDeformedShape->GetIthVertex(indexA);

	//	vector3 contactForce = mForces->mNormalContactForces[indexA];
	//	double pressure = contactForce.length();
	//	
	//	totalPressure += pressure;
	//	cop += pressure * vertexA;
	//}
	//cop /= totalPressure;
	//mCOPLast = mCOP;
	//mCOP = Vector3d(cop.x, cop.y, cop.z);

    ///////// method 3
    MatrixXd lhs = MatrixXd::Zero(4, 3);
    Vector4d rhs = Vector4d::Zero();
    double contactHeight = 0;
    for (set<int>::const_iterator it = contactNodeIndices.begin(); it != contactNodeIndices.end(); ++it)
    {
    	int indexA = *it;

    	vector3 vertexA = mNodes[indexA].mLastPos;
    	vector3 contactForce = mForces->mNormalContactForces[indexA] + mForces->mFrictionContactForces[indexA];

        Vector3d v(vertexA.x, vertexA.y, vertexA.z);
        Vector3d f(contactForce.x, contactForce.y, contactForce.z);

    	rhs.segment<3>(0) += f.cross(v);
        lhs.block<3, 3>(0, 0) += FormulateSkewSymmetricMatrix(f);
        contactHeight += vertexA.y;
    }
    contactHeight /= contactNodeIndices.size();
    lhs(3, 1) = 1;
    rhs(3) = contactHeight;
    mCOPLast = mCOP;
    Vector4d sol = lhs.fullPivLu().solve(rhs);
    mCOP = sol.head(3);
    cop = mCOP;

	//int numContactNodes = static_cast<int>(contactNodeIndices.size());
    vector3 com = GetCenterOfMassLast();
	//vector3 totalTorque(0, 0, 0);
 //   vector3 totalTorque1(0, 0, 0);
    vector3 totalContactForce(0, 0, 0);
    for (set<int>::const_iterator it = contactNodeIndices.begin(); it != contactNodeIndices.end(); ++it)
	{
		int index = *it;
		vector3 pos = mNodes[index].mLastPos;
		vector3 contactForce = mForces->mNormalContactForces[index] + mForces->mFrictionContactForces[index];
        totalContactForce += contactForce;
		//vector3 r = pos - cop;
  //      vector3 r1 = pos - com;
		//vector3 torque = CrossProduct(r, contactForce);
  //      vector3 torque1 = CrossProduct(r1, contactForce);
		//totalTorque += torque;
  //      totalTorque1 += torque1;
	}
    vector3 r2 = cop - com;
    vector3 totalTorque2 = CrossProduct(r2, totalContactForce);
	//LOG(INFO) << "Torque about COP is: " << totalTorque;
 //   LOG(INFO) << "Torque about COM is: " << totalTorque1;
  //  LOG(INFO) << "Torque about COM from COP is: " << totalTorque2;
}

Vector3d DecoSoftObject::GetLinearMomentum()
{
	Vector3d comVel = GetVelOfCOM();
	return mMass * comVel;
}

Vector3d DecoSoftObject::GetAngularMomentum()
{
	vector3 com = GetCenterOfMass();
	int numNodes = static_cast<int>(mNodes.size());
	vector3 angularMomentum(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
		vector3 r = mNodes[i].mPos - com;
		vector3 linearMomentum = mNodes[i].mMass * mNodes[i].mVel;
		angularMomentum += CrossProduct(r, linearMomentum);
	}
	return Vector3d(angularMomentum.x, angularMomentum.y, angularMomentum.z);
}


Vector3d DecoSoftObject::GetCenterOfMass() const
{
    VectorXd pos;
    GetPos(pos);
    Vector3d com = mCenterOfMassMulitplier * pos;
	return com;
}

Vector3d DecoSoftObject::GetCenterOfMassRest() const
{
	VectorXd pos;
	GetPosRest(pos);
	return mCenterOfMassMulitplier * pos;
}

Vector3d DecoSoftObject::GetCenterOfMassLast() const
{
	VectorXd pos;
	GetPosLast(pos);
	Vector3d com = mCenterOfMassMulitplier * pos;
	return com;
}

const MatrixXd& DecoSoftObject::GetCenterOfMassMultiplier() const
{
    return mCenterOfMassMulitplier;
}


void DecoSoftObject::computeMass()
{
	int numVertices = static_cast<int>(mNodes.size());

    double totalMass = 0;
	for (int i = 0; i < numVertices; ++i)
	{
		mNodes[i].mMass = 0;
	}
	for (int i = 0; i < mRestShape->GetNumTetrahedra(); ++i)
	{
		Tetrahedron tet = mRestShape->GetIthTetrahedron(i);
		double volume = tet.CalculateVolume();
		double mass = mRho * volume;
        totalMass += mass;
		for (int ithVertex = 0; ithVertex < 4; ++ithVertex)
		{
			mNodes[tet.mVertIndices[ithVertex]].mMass += mass / 4.0;
		}
	}
	mMassMatrix.SetBlockInfo(3, 3, numVertices, numVertices);
	for (int i = 0; i < numVertices; ++i)
	{
		Eigen::MatrixXd massMatPerVertex = Eigen::MatrixXd::Zero(3, 3);
		massMatPerVertex(0, 0) = massMatPerVertex(1, 1) = massMatPerVertex(2, 2) = mNodes[i].mMass;
		mMassMatrix.SetBlockMatrix(i, i, massMatPerVertex);

	}
    mCenterOfMassMulitplier = MatrixXd::Zero(3, numVertices * 3);
    for (int i = 0; i < numVertices; ++i)
    {
        mCenterOfMassMulitplier(0, i * 3 + 0) = mNodes[i].mMass / totalMass;
        mCenterOfMassMulitplier(1, i * 3 + 1) = mNodes[i].mMass / totalMass;
        mCenterOfMassMulitplier(2, i * 3 + 2) = mNodes[i].mMass / totalMass;
    }
	mMass = totalMass;
}

void DecoSoftObject::precomputation()
{
	computeMass();
	mRestShapeInvertedX.clear();
	for (int i = 0; i < mRestShape->GetNumTetrahedra(); ++i)
	{
		Tetrahedron tet = mRestShape->GetIthTetrahedron(i);

		matrix33 X;
		X[0] = tet.mVertices[1] - tet.mVertices[0];
		X[1] = tet.mVertices[2] - tet.mVertices[0];
		X[2] = tet.mVertices[3] - tet.mVertices[0];
		matrix33 xInvert = X.invert();
		mRestShapeInvertedX.push_back(xInvert);
	}

}

vector3 DecoSoftObject::GetIthVertex(int i) const
{
    return mDeformedShape->GetIthVertex(i);
}
vector3 DecoSoftObject::GetIthVertexRest(int i) const
{
    return mRestShape->GetIthVertex(i);
}
vector3 DecoSoftObject::GetIthVertexVel(int i) const
{
	return mNodes[i].mVel;
}

DecoRenderData* DecoSoftObject::GetRenderData()
{
	mDeformedShape->SetSelectPoint(mSelectedVertices);
    mDeformedShape->SetCenterOfMass(GetCenterOfMass());
	mDeformedShape->SetCenterOfPressure(GetCenterOfPressure());
	return mDeformedShape;
}

void DecoSoftObject::ExertForceOnNode(const VectorXd& force)
{
	int numNodes = static_cast<int>(mNodes.size());
	for (int nodeIndex = 0; nodeIndex < numNodes; ++nodeIndex)
		mNodes[nodeIndex].mForceAccumulator += vector3(force[3 * nodeIndex + 0], force[3 * nodeIndex + 1], force[3 * nodeIndex + 2]);
}

void DecoSoftObject::ExertPerturbationForceOnNode(int nodeIndex, const vector3& force)
{
	ExertForceOnNode(nodeIndex, force);
	mForces->mPerturbationForce[nodeIndex] = force; 
}

void DecoSoftObject::ExertFrictionForceOnNode(int nodeIndex, const vector3& force)
{
	ExertForceOnNode(nodeIndex, force);
	mForces->mFrictionContactForces[nodeIndex] += force;
	mForces->mNodeInContact[nodeIndex] = 1;
}

void DecoSoftObject::ExertNormalContactForceOnNode(int nodeIndex, const vector3& force)
{
	ExertForceOnNode(nodeIndex, force);
	mForces->mNormalContactForces[nodeIndex] += force;
	mForces->mNodeInContact[nodeIndex] = 1;
}

void DecoSoftObject::ExertForceOnNode(int nodeIndex, const vector3& force)
{
	assert(nodeIndex < static_cast<int>(mNodes.size()));
	mNodes[nodeIndex].mForceAccumulator += force;
}

BOOL DecoSoftObject::MoveDeformedShape(const vector3& offset)
{
    int numNodes = mDeformedShape->GetNumVertices();

    for (int i = 0; i < numNodes; ++i)
    {
        vector3 originalPos = mDeformedShape->GetIthVertex(i);
        vector3 newPos = originalPos + offset;
        mNodes[i].mPos = newPos;
        mDeformedShape->SetIthVertex(newPos, i);
    }
    return TRUE;
}

BOOL DecoSoftObject::Move(const vector3& offset)
{
	int numNodes = mDeformedShape->GetNumVertices();

	for (int i = 0; i < numNodes; ++i)
	{
		vector3 originalPos = mRestShape->GetIthVertex(i);
		vector3 newPos = originalPos + offset;
		mNodes[i].mPos = newPos;
		mDeformedShape->SetIthVertex(newPos, i);
        mRestShape->SetIthVertex(newPos, i);
	}
    precomputation();
	return TRUE;

}

BOOL DecoSoftObject::RotateDeformedShape(const vector3& axis, double rad)
{
    int numNodes = mDeformedShape->GetNumVertices();
    matrix33 rotMat = RotateRadMatrix33(axis, rad);
    for (int i = 0; i < numNodes; ++i)
    {
        vector3 originalPos = mDeformedShape->GetIthVertex(i);
        vector3 newPos = rotMat * originalPos;
        mNodes[i].mPos = newPos;
        mDeformedShape->SetIthVertex(newPos, i);
    }
    return TRUE;
}

BOOL DecoSoftObject::Rotate(AXIS axis, float rad, BOOL bAlongGlobal)
{
	int numNodes = mDeformedShape->GetNumVertices();
	matrix44 rotation;
	switch (axis)
	{
	case AXIS_X:
		rotation = RotateRadMatrix44('x', rad);
		break;
	case AXIS_Y:
		rotation = RotateRadMatrix44('y', rad);
		break;
	case AXIS_Z:
		rotation = RotateRadMatrix44('z', rad);
		break;
	} 
	for (int i = 0; i < numNodes; ++i)
	{
		vector3 originalPos = mRestShape->GetIthVertex(i);
		vector3 newPos = TransformPoint(rotation, originalPos);
        mRestShape->SetIthVertex(newPos, i);
		mDeformedShape->SetIthVertex(newPos, i);
        mNodes[i].mPos = newPos;
    }
    precomputation();
	return TRUE;
}
BOOL DecoSoftObject::RotateAlongGlobalVector(vector3 axis, FLOAT rad)
{
	return FALSE;
}

BOOL DecoSoftObject::Stretch(const vector3& fac)
{
    int numNodes = mDeformedShape->GetNumVertices();

    for (int i = 0; i < numNodes; ++i)
    {
        vector3 originalPos = mDeformedShape->GetIthVertex(i);
        vector3 newPos;
        newPos.x = fac.x * originalPos.x;
        newPos.y = fac.y * originalPos.y;
        newPos.z = fac.z * originalPos.z;
        mNodes[i].mPos = newPos;
        mDeformedShape->SetIthVertex(newPos, i);
    }
    //precomputation();
    return TRUE;  
}
BOOL DecoSoftObject::Scale(const vector3& fac)
{
	int numNodes = mDeformedShape->GetNumVertices();

	for (int i = 0; i < numNodes; ++i)
	{
		vector3 originalPos = mDeformedShape->GetIthVertex(i);
		vector3 newPos;
		newPos.x = fac.x * originalPos.x;
		newPos.y = fac.y * originalPos.y;
		newPos.z = fac.z * originalPos.z;
		mNodes[i].mPos = newPos;
		mRestShape->SetIthVertex(newPos, i);
		mDeformedShape->SetIthVertex(newPos, i);
	}
	precomputation();
	return TRUE;
}
BOOL DecoSoftObject::SetPosition(const vector3& pos)
{
	return FALSE;
}


vector3 DecoSoftObject::SampleVelocity(const vector3& pos)
{
	return vector3(0, 0, 0);
}

void DecoSoftObject::ClearPerturbationForce()
{
	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		//mForces->mPerturbationForce[i] = vector3(0, 0, 0);
		mForces->ClearAllForce();
	}
}

void DecoSoftObject::ClearContactForce()
{
	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		mForces->mNodeInContact[i] = 0;
		mForces->mNormalContactForces[i] = vector3(0, 0, 0);
		mForces->mFrictionContactForces[i] = vector3(0, 0, 0);
	}
}
void DecoSoftObject::preUpdate(double timeStep)
{

}
	
void DecoSoftObject::postUpdate(double timeStep)
{

}

matrix33 DecoSoftObject::calculateStressAndDampingForIthTet(int i)
{
	Tetrahedron tet = mDeformedShape->GetIthTetrahedron(i);
	matrix33 P;
	P[0] = tet.mVertices[1] - tet.mVertices[0];
	P[1] = tet.mVertices[2] - tet.mVertices[0];
	P[2] = tet.mVertices[3] - tet.mVertices[0];
	P = P * mRestShapeInvertedX[i];
	matrix33 PTrans = P;
	PTrans.transpose();

	matrix33 strain = 0.5 * (PTrans * P - IdentityMatrix33());
	matrix33 stress = stressStrainRelation(strain);

    return stress;

	matrix33 V;
	V[0] = mNodes[tet.mVertIndices[1]].mVel - mNodes[tet.mVertIndices[0]].mVel;
	V[1] = mNodes[tet.mVertIndices[2]].mVel - mNodes[tet.mVertIndices[0]].mVel;
	V[2] = mNodes[tet.mVertIndices[3]].mVel - mNodes[tet.mVertIndices[0]].mVel;
	V = V * mRestShapeInvertedX[i];
	matrix33 VTrans = V;
	VTrans.transpose();
	matrix33 strainDot = 0.5 * (PTrans * V + VTrans * P);
	matrix33 damping = dampingStrainDotRelation(strainDot);
	return stress + damping;
}

void DecoSoftObject::clearForceAccumulator()
{
	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		mForces->mTotalForces[i] = mNodes[i].mForceAccumulator;
		mNodes[i].mForceAccumulator = vector3(0, 0, 0);
	}
	mForces->Check();
	mbNeedRecalculateForce = true;
}


void DecoSoftObject::accumulateForce(double timeStep)
{
	int numNodes = static_cast<int>(mNodes.size());
    VectorXd gravity = CalculateGravityForce(timeStep);
    if (strstr(name.c_str(), "ground") == NULL) //for the ground plane
    {
        for (int i = 0; i < numNodes; ++i)
        {
            mNodes[i].mForceAccumulator += vector3(gravity[3 * i + 0], gravity[3 * i + 1], gravity[3 * i + 2]);
            mForces->mGravityForce[i] = vector3(gravity[3 * i + 0], gravity[3 * i + 1], gravity[3 * i + 2]);
        }
    }
	Eigen::VectorXd elasticForce = CalculateElasticForce(timeStep);
	for (int i = 0; i < numNodes; ++i)
	{
		mNodes[i].mForceAccumulator += vector3(elasticForce[3 * i], elasticForce[3 * i + 1], elasticForce[3 * i + 2]);
        mForces->mElasticForce[i] = vector3(elasticForce[3 * i + 0], elasticForce[3 * i + 1], elasticForce[3 * i + 2]);

	}


}

matrix33 DecoSoftObject::stressStrainRelation(const matrix33& strain)
{
	matrix33 ret;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (i == j)
			{
				continue;
			}
			else
			{
				ret[i][j] = (1 - 2 * mPoissonRatio) * strain[i][j];
			}
		}
	}
	ret[0][0] = (1 - mPoissonRatio) * strain[0][0] + mPoissonRatio * (strain[1][1] + strain[2][2]);
	ret[1][1] = (1 - mPoissonRatio) * strain[1][1] + mPoissonRatio * (strain[0][0] + strain[2][2]);
	ret[2][2] = (1 - mPoissonRatio) * strain[2][2] + mPoissonRatio * (strain[0][0] + strain[1][1]);
	ret *= mYoungsModulus / ((1 + mPoissonRatio) * (1 - 2 * mPoissonRatio));
	return ret;
}

matrix33 DecoSoftObject::dampingStrainDotRelation(const matrix33& strainDot)
{
	matrix33 ret;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (i == j)
			{
				continue;
			}
			else
			{
				ret[i][j] = (1 - 2 * mDampingConstant2) * strainDot[i][j];
			}
		}
	}
	ret[0][0] = (1 - mDampingConstant2) * strainDot[0][0] + mDampingConstant2 * (strainDot[1][1] + strainDot[2][2]);
	ret[1][1] = (1 - mDampingConstant2) * strainDot[1][1] + mDampingConstant2 * (strainDot[0][0] + strainDot[2][2]);
	ret[2][2] = (1 - mDampingConstant2) * strainDot[2][2] + mDampingConstant2 * (strainDot[0][0] + strainDot[1][1]);
	ret *= mDampingConstant1 / ((1 + mDampingConstant2) * (1 - 2 * mDampingConstant2));
	return ret;

}


void DecoSoftObject::Update(DOUBLE deltaTime)
{
	preUpdate(deltaTime);
	//int numNodes = static_cast<int>(mNodes.size());
	//for (int i = 0; i < numNodes; ++i)
	//{
	//	mNodes[i].mVel += deltaTime * mNodes[i].mForceAccumulator / mNodes[i].mMass;
	//	mNodes[i].mPos += deltaTime * mNodes[i].mVel;
	//	mDeformedShape.SetIthVertex(mNodes[i].mPos, i);
	//}
	mSolver->Solve(deltaTime);
	clearForceAccumulator();
	postUpdate(deltaTime);
}

int DecoSoftObject::GetNumDofs() const
{
	return static_cast<int>(mNodes.size()) * 3;
}

int DecoSoftObject::GetNumUnConstrainedDofs() const
{
    int numConstrainedVertices = 0;
    int numConstraints = static_cast<int>(mConstraints.size());
    for (int i = 0; i < numConstraints; ++i)
    {
        numConstrainedVertices += mConstraints[i]->GetNumConstrainedVertices();
    }
	return (static_cast<int>(mNodes.size()) - numConstrainedVertices) * 3;
}

int DecoSoftObject::GetDim() const
{
	return static_cast<int>(mNodes.size()) * 6;
}

int DecoSoftObject::GetNumElements() const
{
    return mDeformedShape->GetNumTetrahedra();
}

int DecoSoftObject::GetNumNodes() const
{
	return static_cast<int>(mNodes.size());
}

void DecoSoftObject::SetInitialVelocity(const vector3& v)
{
	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		mNodes[i].mVel = v;
	}
}
void DecoSoftObject::SetState(double* state)
{
	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		mNodes[i].mPos.x = state[6 * i];
		mNodes[i].mPos.y = state[6 * i + 1];
		mNodes[i].mPos.z = state[6 * i + 2];

		mNodes[i].mVel.x = state[6 * i + 3];
		mNodes[i].mVel.y = state[6 * i + 4];
		mNodes[i].mVel.z = state[6 * i + 5];
		mDeformedShape->SetIthVertex(mNodes[i].mPos, i);
	}
}
void DecoSoftObject::GetState(double* state) const
{
	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		state[6 * i] = mNodes[i].mPos.x;
		state[6 * i + 1] = mNodes[i].mPos.y;
		state[6 * i + 2] = mNodes[i].mPos.z;

		state[6 * i + 3] = mNodes[i].mVel.x;
		state[6 * i + 4] = mNodes[i].mVel.y;
		state[6 * i + 5] = mNodes[i].mVel.z;


	}
}

Eigen::VectorXd DecoSoftObject::CalculateElasticStress(double timeStep)
{
	int numTets = mDeformedShape->GetNumTetrahedra();
	Eigen::VectorXd elasticStress = Eigen::VectorXd::Zero(6 * numTets);

	for (int i = 0; i < numTets; ++i)
	{
		matrix33 stress = calculateStressAndDampingForIthTet(i);
		elasticStress[6 * i + 0] = stress[0][0];
		elasticStress[6 * i + 1] = stress[1][0];
		elasticStress[6 * i + 2] = stress[2][0];
		elasticStress[6 * i + 3] = stress[1][1];
		elasticStress[6 * i + 4] = stress[2][1];
		elasticStress[6 * i + 5] = stress[2][2];
	}
	return elasticStress;
}


Eigen::VectorXd DecoSoftObject::CalculateGravityForce(double timeStep)
{
	int numVertices = static_cast<int>(mNodes.size());
	Eigen::VectorXd gravity = Eigen::VectorXd::Zero(3 * numVertices);
    if (!bIsGravityMuted)
    {
	    for (int i = 0; i < numVertices; ++i)
	    {
		    gravity[3 * i + 1] -= mGravityConstant * mNodes[i].mMass;
	    }
    }
	return gravity;
}

Eigen::VectorXd DecoSoftObject::CalculateElasticForce(double timeStep)
{
	int numVertices = static_cast<int>(mNodes.size());
	Eigen::VectorXd elasticForce = Eigen::VectorXd::Zero(3 * numVertices);

	int numTets = mDeformedShape->GetNumTetrahedra();
	for (int i = 0; i < numTets; ++i)
	{
		Tetrahedron tet = mDeformedShape->GetIthTetrahedron(i);
		//Tetrahedron tetOriginal = mRestShape->GetIthTetrahedron(i);
		//double originalVolume = tetOriginal.CalculateSignVolume();
		//double deformedVolume = tet.CalculateSignVolume();
		//if (originalVolume * deformedVolume <= 0)
		//{
		//	printf("Warning!!! %dth tetrahedron is inverted\n", i);
		//}

		//matrix33 stress = calculateStressAndDampingForIthTet(i);

        
        MatrixXd faceForces = CalculateElasticForcePerTet(i);
		for (int ithFace = 0; ithFace < 4; ++ithFace)
		{
			//Vector3d fFaceEigenFormat = fromStressToForce(stress, tet, ithFace);
            Vector3d fFaceEigenFormat = faceForces.col(ithFace);
			vector3 fFace(fFaceEigenFormat[0], fFaceEigenFormat[1], fFaceEigenFormat[2]);
			elasticForce[tet.mVertIndices[(ithFace + 0) % 4] * 3 + 0] += fFace.x / 3.0;
			elasticForce[tet.mVertIndices[(ithFace + 0) % 4] * 3 + 1] += fFace.y / 3.0;
			elasticForce[tet.mVertIndices[(ithFace + 0) % 4] * 3 + 2] += fFace.z / 3.0;

			elasticForce[tet.mVertIndices[(ithFace + 1) % 4] * 3 + 0] += fFace.x / 3.0;
			elasticForce[tet.mVertIndices[(ithFace + 1) % 4] * 3 + 1] += fFace.y / 3.0;
			elasticForce[tet.mVertIndices[(ithFace + 1) % 4] * 3 + 2] += fFace.z / 3.0;

			elasticForce[tet.mVertIndices[(ithFace + 2) % 4] * 3 + 0] += fFace.x / 3.0;
			elasticForce[tet.mVertIndices[(ithFace + 2) % 4] * 3 + 1] += fFace.y / 3.0;
			elasticForce[tet.mVertIndices[(ithFace + 2) % 4] * 3 + 2] += fFace.z / 3.0;

		}
	}
	return elasticForce;
}

void DecoSoftObject::CalculateDeriv(double timeStep, double* deriv)
{
	accumulateForce(timeStep);
	if (deriv)
	{
		int numNodes = static_cast<int>(mNodes.size());
		for (int i = 0; i < numNodes; ++i)
		{
			deriv[6 * i] = mNodes[i].mVel.x;
			deriv[6 * i + 1] = mNodes[i].mVel.y;
			deriv[6 * i + 2] = mNodes[i].mVel.z;

			deriv[6 * i + 3] = mNodes[i].mForceAccumulator.x / mNodes[i].mMass;
			deriv[6 * i + 4] = mNodes[i].mForceAccumulator.y / mNodes[i].mMass;
			deriv[6 * i + 5] = mNodes[i].mForceAccumulator.z / mNodes[i].mMass;
		}
	}

}


const BlockSparseMatrix& DecoSoftObject::GetMassMatrix()
{
	mMassMatrix.UpdateCSR();
    return mMassMatrix;
}
const BlockSparseMatrix& DecoSoftObject::GetDampingMatrix()
{
    mDampingMatrix = mDampingConstant1 * GetMassMatrix() + mDampingConstant2 * GetDfDx();
    mDampingMatrix.UpdateCSR();
    return mDampingMatrix;
}

const BlockSparseMatrix& DecoSoftObject::GetDfDx()
{
    int numTets = mDeformedShape->GetNumTetrahedra();
    int numNodes = static_cast<int>(mNodes.size());

    mDfDx.SetBlockInfo(3, 3, numNodes, numNodes);
    mDfDx.SetToZero();
    for (int ithTet = 0; ithTet < numTets; ++ithTet)
    {
        Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
        MatrixXd dfdxPerTet = CalculateDfDXPerTet(ithTet);
        distributeDfDxToVertices(ithTet, dfdxPerTet);
    }
    return mDfDx;
}

const BlockSparseMatrix& DecoSoftObject::GetDfDxFiniteDiff()
{
    const int numComponents = 3;
    int numVertices = static_cast<int>(mNodes.size());
    double epsilon = 0.00001;
    mDfDx.SetBlockInfo(3, 3, numVertices, numVertices);
    mDfDx.SetToZero();
    MatrixXd dfdx = MatrixXd::Zero(3 * numVertices, 3 * numVertices);
    for (int ithVertex = 0; ithVertex < numVertices; ++ithVertex)
    {
        vector3 pos = mNodes[ithVertex].mPos;
        for (int ithComponent = 0; ithComponent < numComponents; ++ithComponent)
        {
            Vector3d posPlus = pos.ConvertToEigen();
            posPlus[ithComponent] += epsilon;
            mDeformedShape->SetIthVertex(posPlus, ithVertex);
            VectorXd elasticForcePlus = CalculateElasticForce(0.033);

            Vector3d posMinus = pos.ConvertToEigen();
            posMinus[ithComponent] -= epsilon;
            mDeformedShape->SetIthVertex(posMinus, ithVertex);
            VectorXd elasticForceMinus = CalculateElasticForce(0.033);

            VectorXd deriv = (elasticForcePlus - elasticForceMinus) / 2.0 / epsilon;
            mDeformedShape->SetIthVertex(pos, ithVertex);

            dfdx.col(ithVertex * numComponents + ithComponent) = deriv;
        }
    }
    mDfDx.ConvertFromEigenMatrix(dfdx);
    return mDfDx;
}

MatrixXd DecoSoftObject::CalculateElasticForcePerTet(int ithTet)
{
    MatrixXd ret(3, 4);
    int numTets = mDeformedShape->GetNumTetrahedra();

    Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
    Tetrahedron tetOriginal = mRestShape->GetIthTetrahedron(ithTet);
    double originalVolume = tetOriginal.CalculateSignVolume();
    double deformedVolume = tet.CalculateSignVolume();
    if (originalVolume * deformedVolume <= 0)
    {
        printf("Warning!!! %dth tetrahedron is inverted\n", ithTet);
    }

    matrix33 stress = calculateStressAndDampingForIthTet(ithTet);
    for (int ithFace = 0; ithFace < 4; ++ithFace)
    {
        Vector3d fFaceEigenFormat = fromStressToForce(stress, tet, ithFace);
        ret.col(ithFace) = fFaceEigenFormat;
    }

    return ret;
}

MatrixXd DecoSoftObject::CalculateDfDXPerTet(int ithTet)
{
    const int numVPerTet = 4;
    const int numComponentPerV = 3;
    const int numFPerTet = 4;
    MatrixXd ret = MatrixXd::Zero(12, 12);
    Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);

    matrix33 stress = calculateStressAndDampingForIthTet(ithTet);
    for (int ithFace = 0; ithFace < numFPerTet; ++ithFace)
    {
        vector3 an = calculateAreaNormal(tet, ithFace);
        for (int ithVertex = 0; ithVertex < numVPerTet; ++ithVertex)
        {
            for (int ithComponent = 0; ithComponent < numComponentPerV; ++ithComponent)
            {
                matrix33 dEpsilonDx = calculateDStrainDXComponent(ithTet, ithVertex, ithComponent);
                matrix33 dStressDxFirstPart = stressStrainRelation(dEpsilonDx);
                vector3 dfFaceDxFirstPart = -0.5 * dStressDxFirstPart * an;

                vector3 dfFaceDxSecondPart = -1 * stress * calculateDAreaNormalDxComponent(ithTet, ithFace, ithVertex, ithComponent);
                ret(ithFace * numComponentPerV + ithComponent, ithVertex * numComponentPerV + 0) = dfFaceDxFirstPart.x + dfFaceDxSecondPart.x;
                ret(ithFace * numComponentPerV + ithComponent, ithVertex * numComponentPerV + 1) = dfFaceDxFirstPart.y + dfFaceDxSecondPart.y;
                ret(ithFace * numComponentPerV + ithComponent, ithVertex * numComponentPerV + 2) = dfFaceDxFirstPart.z + dfFaceDxSecondPart.z;
                
            }
        }
    }

    return ret;
}

MatrixXd DecoSoftObject::CalculateDfDxPerTetFiniteDiff(int ithTet)
{
    const int numVPerTet = 4;
    const int numComponentPerV = 3;
    const int numFPerTet = 4;
    MatrixXd ret = MatrixXd::Zero(12, 12);

    Tetrahedron originalTet = mDeformedShape->GetIthTetrahedron(ithTet);

    double epsilon = 0.00001;

    for (int ithVertex = 0; ithVertex < numVPerTet; ++ithVertex)
    {
        Vector3d pos = originalTet.mVertices[ithVertex].ConvertToEigen();
        
        for (int ithComponent = 0; ithComponent < numComponentPerV; ++ithComponent)
        {
            Vector3d posPlus = pos;
            posPlus[ithComponent] += epsilon;
            mDeformedShape->SetIthVertex(posPlus, originalTet.mVertIndices[ithVertex]);
            MatrixXd forcePlus = CalculateElasticForcePerTet(ithTet);

            Vector3d posMinus = pos;
            posMinus[ithComponent] -= epsilon;
            mDeformedShape->SetIthVertex(posMinus, originalTet.mVertIndices[ithVertex]);
            MatrixXd forceMinus = CalculateElasticForcePerTet(ithTet);

            MatrixXd deriv = (forcePlus - forceMinus) / 2.0 / epsilon;
            
            for (int ithFace = 0; ithFace < numFPerTet; ++ithFace)
            {
                ret(ithFace * numComponentPerV + ithComponent, ithVertex * numComponentPerV + 0) = deriv(0, ithFace);
                ret(ithFace * numComponentPerV + ithComponent, ithVertex * numComponentPerV + 1) = deriv(1, ithFace);
                ret(ithFace * numComponentPerV + ithComponent, ithVertex * numComponentPerV + 2) = deriv(2, ithFace);

            }

            for (int i = 0; i < numVPerTet; ++i)
            {
                mDeformedShape->SetIthVertex(originalTet.mVertices[i], originalTet.mVertIndices[i]);
            }
        }
    }


    return ret;

}

matrix33 DecoSoftObject::calculateDStrainDXComponent(int ithTet, int ithVertex, int ithComponent)
{
    Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
    const matrix33& invX = mRestShapeInvertedX[ithTet];

    matrix33 P;
    P[0] = tet.mVertices[1] - tet.mVertices[0];
    P[1] = tet.mVertices[2] - tet.mVertices[0];
    P[2] = tet.mVertices[3] - tet.mVertices[0];

    P *= invX;

    matrix33 PTranspose = P;
    PTranspose.transpose();

    matrix33 dPdPComponent;
    if (ithVertex == 0)
    {
        dPdPComponent.zero();
        for (int i = 0; i < 3; ++i)
        {
            dPdPComponent[i][ithComponent] = -1;
        }
        dPdPComponent *= invX;
    }
    else
    {
        dPdPComponent.zero();
        dPdPComponent[ithVertex - 1][ithComponent] = 1;

        dPdPComponent *= invX;
    }

    matrix33 dPdPComponentTranspose = dPdPComponent;
    dPdPComponentTranspose.transpose();

    matrix33 ret = PTranspose * dPdPComponent + dPdPComponentTranspose * P;
    return ret;
}

vector3 DecoSoftObject::calculateDAreaNormalDxComponent(int ithTet, int ithFace, int ithVertex, int ithComponent)
{
    Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);

    vector3 p01 = tet.mVertices[(1 + ithFace) % 4] - tet.mVertices[ithFace];
    vector3 p02 = tet.mVertices[(2 + ithFace) % 4] - tet.mVertices[ithFace];
    vector3 p03 = tet.mVertices[(3 + ithFace) % 4] - tet.mVertices[ithFace];
    vector3 ret(0, 0, 0);
    vector3 an = CrossProduct(p01, p02);
    if (ithVertex == (3 + ithFace) % 4)
    {
        return vector3(0, 0, 0);
    }
    else if (ithVertex == ithFace)
    {
        matrix33 matPart1 = FormulateSkewSymmetricMatrix(p01);
        vector3 vecPart2 = p02;
        matrix33 matPart2;
        vector3 vecPart1;
        switch (ithComponent)
        {
        case 0:
            vecPart1 = vector3(-1, 0, 0);
            matPart2[0][0] = 0;     matPart2[1][0] = 0;     matPart2[2][0] = 0;
            matPart2[0][1] = 0;     matPart2[1][1] = 0;     matPart2[2][1] = 1;
            matPart2[0][2] = 0;     matPart2[1][2] = -1;    matPart2[2][2] = 0;
            break;
        case 1:
            vecPart1 = vector3(0, -1, 0);
            matPart2[0][0] = 0;     matPart2[1][0] = 0;     matPart2[2][0] = -1;
            matPart2[0][1] = 0;     matPart2[1][1] = 0;     matPart2[2][1] = 0;
            matPart2[0][2] = 1;     matPart2[1][2] = 0;     matPart2[2][2] = 0;
            break;
        case 2:
            vecPart1 = vector3(0, 0, -1);
            matPart2[0][0] = 0;     matPart2[1][0] = 1;     matPart2[2][0] = 0;
            matPart2[0][1] = -1;    matPart2[1][1] = 0;     matPart2[2][1] = 0;
            matPart2[0][2] = 0;     matPart2[1][2] = 0;     matPart2[2][2] = 0;
            break;
        default:
            LOG(FATAL) << "Component " << ithComponent << " is not defined for a 3D vector.";
            break;
        }
        ret = matPart1 * vecPart1 + matPart2 * vecPart2;
    }
    else if (ithVertex == (1 + ithFace) % 4)
    {
        matrix33 matPart2;
        vector3 vecPart2 = p02;

        switch (ithComponent)
        {
        case 0:
            matPart2[0][0] = 0;     matPart2[1][0] = 0;     matPart2[2][0] = 0;
            matPart2[0][1] = 0;     matPart2[1][1] = 0;     matPart2[2][1] = -1;
            matPart2[0][2] = 0;     matPart2[1][2] = 1;     matPart2[2][2] = 0;
            break;
        case 1:
            matPart2[0][0] = 0;     matPart2[1][0] = 0;     matPart2[2][0] = 1;
            matPart2[0][1] = 0;     matPart2[1][1] = 0;     matPart2[2][1] = 0;
            matPart2[0][2] = -1;    matPart2[1][2] = 0;     matPart2[2][2] = 0;
            break;
        case 2:
            matPart2[0][0] = 0;     matPart2[1][0] = -1;    matPart2[2][0] = 0;
            matPart2[0][1] = 1;     matPart2[1][1] = 0;     matPart2[2][1] = 0;
            matPart2[0][2] = 0;     matPart2[1][2] = 0;     matPart2[2][2] = 0;
            break;
        default:
            LOG(FATAL) << "Component " << ithComponent << " is not defined for a 3D vector.";
            break;
        }
        ret = matPart2 * vecPart2; //part 1 is zero

    }
    else if (ithVertex == (2 + ithFace) % 4)
    {
        matrix33 matPart1 = FormulateSkewSymmetricMatrix(p01);
        vector3 vecPart1;
        switch (ithComponent)
        {
        case 0:
            vecPart1 = vector3(1, 0, 0);
            break;
        case 1:
            vecPart1 = vector3(0, 1, 0);
            break;
        case 2:
            vecPart1 = vector3(0, 0, 1);
            break;
        default:
            LOG(FATAL) << "Component " << ithComponent << " is not defined for a 3D vector.";
            break;
        }
        ret = matPart1 * vecPart1; // part 2 is zero

    }
    if (DotProduct(p03, an) > 0)
    {
        ret = -ret;
    }
    return ret;
}

vector3 DecoSoftObject::calculateAreaNormal(const Tetrahedron& tet, int ithFace)
{
    vector3 p01 = tet.mVertices[(1 + ithFace) % 4] - tet.mVertices[ithFace];
    vector3 p02 = tet.mVertices[(2 + ithFace) % 4] - tet.mVertices[ithFace];
    vector3 p03 = tet.mVertices[(3 + ithFace) % 4] - tet.mVertices[ithFace];
    vector3 an = CrossProduct(p01, p02);
    if (DotProduct(p03, an) > 0)
    {
        an = -an;
    }
    return an;
}

void DecoSoftObject::distributeDfDxToVertices(int ithTet, const MatrixXd& dfdxPerTet)
{
    const int numFPerTet = 4;
    const int numVPerTet = 4;
    Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
    for (int ithFace = 0; ithFace < numFPerTet; ++ithFace)
    {
        int a = tet.mVertIndices[ithFace];
        int b = tet.mVertIndices[(1 + ithFace) % 4];
        int c = tet.mVertIndices[(2 + ithFace) % 4];
        for (int ithVertex = 0; ithVertex < numVPerTet; ++ithVertex)
        {
            int vertexId = tet.mVertIndices[ithVertex];
            MatrixXd toDistribute = dfdxPerTet.block<3, 3>(ithFace * 3, ithVertex * 3) / 3.0;
            mDfDx.AddBlockMatrixTo(a, vertexId, toDistribute.transpose());
            mDfDx.AddBlockMatrixTo(b, vertexId, toDistribute.transpose());
            mDfDx.AddBlockMatrixTo(c, vertexId, toDistribute.transpose());
        }
    }
}

const BlockSparseMatrix& DecoSoftObject::GetAugmentedMassMatrix(double timeStep)
{
	mAugmentedMassMatrix = mMassMatrix + /*timeStep * GetDampingMatrix() */ -(timeStep * timeStep) * GetDfDx();
	mAugmentedMassMatrix.UpdateCSR();
	return mAugmentedMassMatrix;
}

BlockSparseMatrix& DecoSoftObject::GetReferenceMassMatrix()
{
	mMassMatrix.UpdateCSR();
	return mMassMatrix; //hack for compile, not used
}

void DecoSoftObject::PushForceAccumulator()
{
	int numNodes = GetNumNodes();
	for (int i = 0; i < numNodes; ++i)
	{
		mNodes[i].mBackupForceAccumulator = mNodes[i].mForceAccumulator;
	}
    mForces->Push();
}
void DecoSoftObject::PopForceAccumulator()
{
	int numNodes = GetNumNodes();
	for (int i = 0; i < numNodes; ++i)
	{
		mNodes[i].mForceAccumulator = mNodes[i].mBackupForceAccumulator;
	}
    mForces->Pop();
}

DecoSoftObjectNode DecoSoftObject::GetNode(int ithNode) const
{
	return mNodes[ithNode];
}

double DecoSoftObject::GetNodalMass(int ithNode) const
{
    return mNodes[ithNode].mMass;
}

void DecoSoftObject::GetForce(Eigen::VectorXd& force) const
{
	int numVertices = static_cast<int>(mNodes.size());
	force = Eigen::VectorXd(3 * numVertices);

	for (int i = 0; i < numVertices; ++i)
	{
		force(3 * i + 0) = mNodes[i].mForceAccumulator.x;
		force(3 * i + 1) = mNodes[i].mForceAccumulator.y;
		force(3 * i + 2) = mNodes[i].mForceAccumulator.z;
	}

}

void DecoSoftObject::GetVel(Eigen::VectorXd& vel) const
{
	int numVertices = static_cast<int>(mNodes.size());
	vel = Eigen::VectorXd(3 * numVertices);
	for (int i = 0; i < numVertices; ++i)
	{
		vel(3 * i + 0) = mNodes[i].mVel.x;
		vel(3 * i + 1) = mNodes[i].mVel.y;
		vel(3 * i + 2) = mNodes[i].mVel.z;
	}
}

double DecoSoftObject::GetMass() const
{
	return mMass;
}

void DecoSoftObject::GetPos(Eigen::VectorXd& pos) const
{
	int numVertices = static_cast<int>(mNodes.size());
	pos = Eigen::VectorXd(3 * numVertices);
	for (int i = 0; i < numVertices; ++i)
	{
		pos(3 * i + 0) = mNodes[i].mPos.x;
		pos(3 * i + 1) = mNodes[i].mPos.y;
		pos(3 * i + 2) = mNodes[i].mPos.z;

	}
}

void DecoSoftObject::GetPosRest(Eigen::VectorXd& pos) const
{
	int numVertices = static_cast<int>(mNodes.size());
	pos = Eigen::VectorXd(3 * numVertices);
	for (int i = 0; i < numVertices; ++i)
	{
		vector3 postRest = mRestShape->GetIthVertex(i);
		pos(3 * i + 0) = postRest.x;
		pos(3 * i + 1) = postRest.y;
		pos(3 * i + 2) = postRest.z;

	}
}

void DecoSoftObject::GetPosLast(Eigen::VectorXd& pos) const
{
	int numVertices = static_cast<int>(mNodes.size());
	pos = Eigen::VectorXd(3 * numVertices);
	for (int i = 0; i < numVertices; ++i)
	{
		vector3 postRest = mRestShape->GetIthVertex(i);
		pos(3 * i + 0) = mNodes[i].mLastPos.x;
		pos(3 * i + 1) = mNodes[i].mLastPos.y;
		pos(3 * i + 2) = mNodes[i].mLastPos.z;

	}
}

void DecoSoftObject::SetVel(const Eigen::VectorXd& vel)
{
	int numVertices = static_cast<int>(mNodes.size());
	for (int i = 0; i < numVertices; ++i)
	{
		mNodes[i].mVel.x = vel[3 * i];
		mNodes[i].mVel.y = vel[3 * i + 1];
		mNodes[i].mVel.z = vel[3 * i + 2];
	}
}
void DecoSoftObject::SetPos(const Eigen::VectorXd& pos)
{
	int numVertices = static_cast<int>(mNodes.size());
	for (int i = 0; i < numVertices; ++i)
	{
		mNodes[i].mLastPos = mNodes[i].mPos;
		mNodes[i].mPos.x = pos[3 * i];
		mNodes[i].mPos.y = pos[3 * i + 1];
		mNodes[i].mPos.z = pos[3 * i + 2];
		mDeformedShape->SetIthVertex(mNodes[i].mPos, i);
	}
}

void DecoSoftObject::SetIthPos(int ithVertex, const Vector3d& pos)
{
    mNodes[ithVertex].mLastPos = mNodes[ithVertex].mPos;
    mNodes[ithVertex].mPos = pos;

    mDeformedShape->SetIthVertex(mNodes[ithVertex].mPos, ithVertex);
}

Vector3d DecoSoftObject::fromStressToForce(const matrix33& stress, const Tetrahedron& tet, int ithFace)
{
	Matrix3d stressEigenFormat;
	stressEigenFormat << stress[0][0], stress[1][0], stress[2][0],
						 stress[0][1], stress[1][1], stress[2][1],
						 stress[0][2], stress[1][2], stress[2][2];
	return fromStressToForce(stressEigenFormat, tet, ithFace);
}

Vector3d DecoSoftObject::fromStressToForce(const Matrix3d& stress, const Tetrahedron& tet, int ithFace)
{
    vector3 an = calculateAreaNormal(tet, ithFace);
    Vector3d fFace = -stress * Vector3d(an.x, an.y, an.z);
//	Vector3d fFace = -(area * stress * normal);
	return fFace;
}

void DecoSoftObject::SetRenderOption(SoftObjRenderOption option)
{
	mRenderOption = option;
	mRestShape->SetRenderOption(option);
	mDeformedShape->SetRenderOption(option);
}



void DecoSoftObject::RemoveConstrainedDof(BlockSparseMatrix& mat) const
{
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		mConstraints[i]->RemoveConstrainedDof(mat);
	}

}
void DecoSoftObject::RemoveConstrainedDof(VectorXd& vec) const
{
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		mConstraints[i]->RemoveConstrainedDof(vec);
	}
}

void DecoSoftObject::SetZeroConstrainedDof(VectorXd& vec) const
{
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		mConstraints[i]->SetZeroConstrainedDof(vec);
	}
}

void DecoSoftObject::SetZeroColumnConstrainedDof(MatrixXd& mat) const
{
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		mConstraints[i]->SetZeroColumnConstrainedDof(mat);
	}

}

void DecoSoftObject::RemoveColConstrainedDof(MatrixXd& mat) const
{
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		mConstraints[i]->RemoveColConstrainedDof(mat);
	}
}

void DecoSoftObject::RemoveRowConstrainedDof(MatrixXd& mat) const
{
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		mConstraints[i]->RemoveRowConstrainedDof(mat);
	}
}

void DecoSoftObject::AddConstrainedDof(VectorXd& vec) const
{
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		mConstraints[i]->AddConstrainedDof(vec);
	}
}

void DecoSoftObject::ClearAllConstraints()
{
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		delete mConstraints[i];
	}
	mConstraints.clear();
}

bool DecoSoftObject::SelectPoints(const vector3& origin, const vector3& dir, double& depth, int& vertexId)
{
	int numNodes = static_cast<int>(mNodes.size());
	double minDist = 1e30;
	
	for (int i = 0; i < numNodes; ++i)
	{
		double rayDepth;
		double distance = PointRayDistance(mNodes[i].mPos, origin, dir, rayDepth);
		if (distance < minDist)
		{
			minDist = distance;
			depth = rayDepth;
			vertexId = i;
		}
	}
	const double threshold = 0.1;
	if (minDist < threshold)
		return true;
	else
		return false;
}

void DecoSoftObject::SetSelectVertex(int id, bool bAppend)
{
	if (!bAppend)
		mSelectedVertices.clear();
	vector<int>::iterator it = find(mSelectedVertices.begin(), mSelectedVertices.end(), id);
	if (it != mSelectedVertices.end())
	{
		mSelectedVertices.erase(it);
		LOG(INFO) << "Deselect vertex " << id << ": (" << mNodes[id].mPos.x << ", " << mNodes[id].mPos.y << ", " << mNodes[id].mPos.z << ")";
	}
	else
	{
		mSelectedVertices.push_back(id);
		LOG(INFO) << "Select vertex " << id << ": (" << mNodes[id].mPos.x << ", " << mNodes[id].mPos.y << ", " << mNodes[id].mPos.z << ")";
	}
	
}


void DecoSoftObject::ToggleForceRenderOption(ForceRecorderRenderType type)
{
	mForces->ToggleRenderType(type);
}

void DecoSoftObject::DrawForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	mForces->Render(RI, Lights, numLight, DrawType);
}

void DecoSoftObject::AddConstraint(SoftBodyVertexConstraint* constraint)
{
	constraint->SetObject(this);
	mConstraints.push_back(constraint);
}

void DecoSoftObject::RemoveAllConstraints()
{
	mConstraints.clear();
}

void DecoSoftObject::ConstructTestConstraint()
{
	if (strstr(name.c_str(), "bar") == NULL)
		return;
	SoftBodyVertexConstraint* constraint = new SoftBodyVertexConstraint(this);

	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		if (mNodes[i].mPos.x < -9.99)
		{
			constraint->AddVertex(i);
		}
	}
	mConstraints.push_back(constraint);
}

void DecoSoftObject::RecordMovie(DecoArchive& Ar, double playerVersion) const
{
    Ar << mNodes;
	mForces->SetPlayerVersion(playerVersion);
	Ar << *mForces;

}

void DecoSoftObject::SetInitialPoseForMovie(DecoArchive& Ar, double playerVersion)
{
	PlayBackFrame frame(playerVersion);
	Ar >> frame;
    mPlaybackMovie.push_back(frame);
	int numNodes = static_cast<int>(frame.mNodes.size());
	for (int i = 0; i < numNodes; ++i)
		mDeformedShape->SetIthVertex(frame.mNodes[i].mPos, i);
}

void DecoSoftObject::CacheMovie(DecoArchive& Ar, double playerVersion)
{
	PlayBackFrame frame(playerVersion);
	Ar >> frame;

	mPlaybackMovie.push_back(frame);
}

void DecoSoftObject::PlaybackMovie(int ithFrame)
{
	int numCachedFrames = static_cast<int>(mPlaybackMovie.size());
	if (numCachedFrames <= ithFrame)
	{
		LOG(WARNING) << ithFrame << "th frame of " << name << " is not cached. Playback failed.";
		return;
	}
	const vector<DecoSoftObjectNode>& node = mPlaybackMovie[ithFrame].mNodes;
	mNodes = node;

	mForces->CopyContent(*(mPlaybackMovie[ithFrame].mForces));
	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		mDeformedShape->SetIthVertex(mNodes[i].mPos, i);
		if (ithFrame - 1 >= 0)
		{
			const DecoSoftObjectNode& lastNode = mPlaybackMovie[ithFrame - 1].mNodes[i];
			mNodes[i].mLastPos = lastNode.mPos;
		}
	}
}

void DecoSoftObject::Serialize(DecoArchive& Ar) const
{
	DecoSceneObject::Serialize(Ar);

	Ar << mFrictionCoeff;
	Ar << mRho;
	Ar << mPoissonRatio;
	Ar << mYoungsModulus;
	Ar << mDampingConstant1;
	Ar << mDampingConstant2;
	Ar << objID;
	Ar << mNodes;
	int numPoints = mRestShape->GetNumVertices();
	Ar << numPoints;
	for (int i = 0; i < numPoints; ++i)
		Ar << mRestShape->GetIthVertex(i);
}

void DecoSoftObject::Deserialize(DecoArchive& Ar)
{
	DecoSceneObject::Deserialize(Ar);
	Ar >> mFrictionCoeff;
	Ar >> mRho;
	Ar >> mPoissonRatio;
	Ar >> mYoungsModulus;
	Ar >> mDampingConstant1;
	Ar >> mDampingConstant2;
	Ar >> objID;
	Ar >> mNodes;

	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		mDeformedShape->SetIthVertex(mNodes[i].mPos, i);
	}

	int numPoints;
	Ar >> numPoints;
	for (int i = 0; i < numPoints; ++i)
	{
		vector3 ithVertex;
		Ar >>ithVertex;
		mRestShape->SetIthVertex(ithVertex, i);
	}
	precomputation();
}

