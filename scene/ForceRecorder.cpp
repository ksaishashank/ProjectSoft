#include "ForceRecorder.h"
#include "ActiveSoftBody.h"
#include "utility/ConvexPolygon.h"

ForceRecorder::ForceRecorder() : mType(0)
{

}
ForceRecorder::~ForceRecorder()
{

}

void ForceRecorder::ClearAllForce()
{
	int numNodes = static_cast<int>(mObj->mNodes.size());

	for (int i = 0; i < numNodes; ++i)
	{
		mNormalContactForces[i] = vector3(0, 0, 0); 
		mFrictionContactForces[i] = vector3(0, 0, 0);
		mTotalForces[i] = vector3(0, 0, 0);
		mMuscleForce[i] = vector3(0, 0, 0);
		mGravityForce[i] = vector3(0, 0, 0);
		mElasticForce[i] = vector3(0, 0, 0);
		mNodeInContact[i] = 0;
		mPerturbationForce[i] = vector3(0, 0, 0);
	}

}

void ForceRecorder::CopyContent(const ForceRecorder& rhs)
{

	mNormalContactForces = rhs.mNormalContactForces; 
	mFrictionContactForces = rhs.mFrictionContactForces;
	mTotalForces = rhs.mTotalForces;
	mMuscleForce = rhs.mMuscleForce;
	mGravityForce = rhs.mGravityForce;
	mElasticForce = rhs.mElasticForce;
	mNodeInContact = rhs.mNodeInContact;
	mContactPolygon = rhs.mContactPolygon;
	mPerturbationForce = rhs.mPerturbationForce;
}
void ForceRecorder::SetRecordingObject(DecoSoftObject* obj)
{
	mObj = obj;
}
void ForceRecorder::SetRenderType(ForceRecorderRenderType type)
{
	mType = type;
}

void ForceRecorder::SetPlayerVersion(double version)
{
	mVersion = version;
}

void ForceRecorder::ToggleRenderType(ForceRecorderRenderType type)
{
	mType = mType ^ type;
}

void ForceRecorder::SetNumNodes(int numNodes)
{
	mGravityForce.resize(numNodes);
	mMuscleForce.resize(numNodes);
	mElasticForce.resize(numNodes);
	mNormalContactForces.resize(numNodes);
	mFrictionContactForces.resize(numNodes);
	mTotalForces.resize(numNodes);
	mNodeInContact.resize(numNodes);
	mPerturbationForce.resize(numNodes);
}

vector3 ForceRecorder::GetNetInternalForce() const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	vector3 force(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
		force += mElasticForce[i] + mMuscleForce[i];
	}

	return force;
}
vector3 ForceRecorder::GetNetExternalForce() const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	vector3 force(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
		force += mGravityForce[i] + mNormalContactForces[i] + mFrictionContactForces[i];
	}
	return force;
}
vector3 ForceRecorder::GetNetContactForce() const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	vector3 force(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
		force += mNormalContactForces[i] + mFrictionContactForces[i];
	}
	return force;
}
vector3 ForceRecorder::GetNetForce() const
{
	return GetNetInternalForce() + GetNetExternalForce();
}
vector3 ForceRecorder::GetNetInternalTorque() const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	vector3 com = mObj->GetCenterOfMassLast();
	vector3 torque(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
		vector3 pos = mObj->mNodes[i].mLastPos;
		vector3 r = pos - com;
		torque += CrossProduct(r, mElasticForce[i] + mMuscleForce[i]);
	}
	return torque;
}
vector3 ForceRecorder::GetNetExternalTorque() const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	vector3 com = mObj->GetCenterOfMassLast();
	vector3 torque(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
		vector3 pos = mObj->mNodes[i].mLastPos;
		vector3 r = pos - com;
		torque += CrossProduct(r, mGravityForce[i] + mNormalContactForces[i] + mFrictionContactForces[i]);
	}
	return torque;
}
vector3 ForceRecorder::GetNetTorque() const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	vector3 com = mObj->GetCenterOfMassLast();
	vector3 torque(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
		vector3 pos = mObj->mNodes[i].mLastPos;
		vector3 r = pos - com;
		torque += CrossProduct(r, mTotalForces[i]);
	}
	return torque;
}

vector3 ForceRecorder::GetNetContactTorque() const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	vector3 com = mObj->GetCenterOfMassLast();
    //LOG(INFO) << "com: " << com;
	vector3 torque(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
        if (mNodeInContact[i])
        {
		    vector3 pos = mObj->mNodes[i].mLastPos;
		    vector3 r = pos - com;
		    torque += CrossProduct(r, mNormalContactForces[i] + mFrictionContactForces[i]);
            //LOG(INFO) << i << "th contact force: " << mNormalContactForces[i] + mFrictionContactForces[i];
        }
	}
	return torque;
}

void ForceRecorder::Check()
{	
	int numNodes = static_cast<int>(mObj->mNodes.size());

	vector3 internalForce(0, 0, 0);
	vector3 externalForce(0, 0, 0);
	vector3 netForce(0, 0, 0);
	for (int i = 0; i < numNodes; ++i)
	{
		vector3 totalForce = mElasticForce[i] + mMuscleForce[i] + mGravityForce[i] + mNormalContactForces[i] + mFrictionContactForces[i];
		if ((mTotalForces[i] - totalForce).length() > 1e-6)
		{
			LOG(INFO) << "Force does not agree: " << i <<  ": " << mTotalForces[i] << " " << totalForce;
		}
		netForce += mTotalForces[i];
		internalForce += mElasticForce[i] + mMuscleForce[i];
		externalForce += mGravityForce[i] + mNormalContactForces[i] + mFrictionContactForces[i];
	}
}
void ForceRecorder::Render(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	CHECK(mObj);
    ActiveSoftBody* activeBody = dynamic_cast<ActiveSoftBody*>(mObj);
    //if (!activeBody)
    //    return;
	ConvexPolygon poly;
	poly.SetVertices(mContactPolygon);
	poly.Render(RI);

	drawPerturbationForce(RI);

	if (mType & FRRT_Contact)
	{
		drawContactForce(RI);
	}
	if (mType & FRRT_Elastic)
	{
		drawElasticForce(RI);
		//drawInternalForce(RI);
	}
	if (mType & FRRT_Gravity)
	{
		drawContactForceOnCOP(RI);
		//drawInternalTorque(RI);

	}
	if (mType & FRRT_Muscle)
	{
		drawTotalForce(RI);
		//drawTotalTorque(RI);
		//drawMuscleForce(RI);
	}
	if (mType & FRRT_Total)
	{
	//	drawTotalForce(RI);
		drawTotalTorque(RI);
	}

}

void ForceRecorder::Push()
{
    mBackupNormalContactForces = mNormalContactForces; 
    mBackupFrictionContactForces = mFrictionContactForces;
    mBackupTotalForces = mTotalForces;
    
    mBackupMuscleForce = mMuscleForce;
    mBackupGravityForce = mGravityForce;
    mBackupElasticForce = mElasticForce;
    mBackupContactPolygon = mContactPolygon;
    mBackupPerturbationForce = mPerturbationForce;

    mBackupNodeInContact = mNodeInContact;
}

void ForceRecorder::Pop()
{
    mNormalContactForces = mBackupNormalContactForces; 
    mFrictionContactForces = mBackupFrictionContactForces;
    mTotalForces = mBackupTotalForces;

    mMuscleForce = mBackupMuscleForce;
    mGravityForce = mBackupGravityForce;
    mElasticForce = mBackupElasticForce;
    mContactPolygon = mBackupContactPolygon;
    mPerturbationForce = mBackupPerturbationForce;

    mNodeInContact = mBackupNodeInContact;
}

void ForceRecorder::drawContactForceOnCOP(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
    const double scale = 1e-1;
    vector<vector3> allVertices;
    vector3 totalForce = GetNetContactForce();
    vector3 start = mObj->GetCenterOfPressure();
    vector3 end = start + scale * (totalForce);
    allVertices.push_back(start);
    allVertices.push_back(end);

    DecoVertexBuffer vb;
    vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
    RI->SetColor(DecoColor(0xff00ffff));
    RI->SetLineWidth(2);
    RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);

}

void ForceRecorder::drawContactForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	const double scale = 1e-1;
	vector<vector3> allVertices;

	for (int i = 0; i < numNodes; ++i)
	{
		if (!mNodeInContact[i])
			continue;
		vector3 start = mObj->mNodes[i].mPos;
		vector3 end = start + scale * (mNormalContactForces[i] + mFrictionContactForces[i]);
		allVertices.push_back(start);
		allVertices.push_back(end);

		if (allVertices.size())
		{
			DecoVertexBuffer vb;
			vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
			RI->SetColor(DecoColor(0xff00ffff));
			RI->SetLineWidth(2);
			RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
		}
	}

}
void ForceRecorder::drawGravityForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	const double scale = 1e-1;
	vector<vector3> allVertices;

	for (int i = 0; i < numNodes; ++i)
	{
		vector3 start = mObj->mNodes[i].mPos;
		vector3 end = start + scale * (mGravityForce[i]);
		allVertices.push_back(start);
		allVertices.push_back(end);

		if (allVertices.size())
		{
			DecoVertexBuffer vb;
			vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
			RI->SetColor(DecoColor(0xff7f7f7f));
			RI->SetLineWidth(2);
			RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
		}
	}

}

void ForceRecorder::drawPerturbationForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	if (mPerturbationForce.empty())
		return;
	int numNodes = static_cast<int>(mObj->mNodes.size());
	const double scale = 1e-1;
	vector<vector3> allVertices;

	for (int i = 0; i < numNodes; ++i)
	{
		vector3 start = mObj->mNodes[i].mPos;
		vector3 end = start + scale * (mPerturbationForce[i]);
		allVertices.push_back(start);
		allVertices.push_back(end);

		if (allVertices.size())
		{
			DecoVertexBuffer vb;
			vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
			RI->SetColor(DecoColor(0xff0000ff));
			RI->SetLineWidth(2);
			RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
		}
	}
}

void ForceRecorder::drawElasticForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	const double scale = 1e-2;
	vector<vector3> allVertices;

	for (int i = 0; i < numNodes; ++i)
	{
		vector3 start = mObj->mNodes[i].mPos;
		vector3 end = start + scale * (mElasticForce[i]);
		allVertices.push_back(start);
		allVertices.push_back(end);

		if (allVertices.size())
		{
			DecoVertexBuffer vb;
			vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
			RI->SetColor(DecoColor(0xff00ff00));
			RI->SetLineWidth(2);
			RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
		}
	}

}
void ForceRecorder::drawMuscleForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	const double scale = 1e-2;
	vector<vector3> allVertices;

	for (int i = 0; i < numNodes; ++i)
	{
		vector3 start = mObj->mNodes[i].mPos;
		vector3 end = start + scale * (mMuscleForce[i]);
		allVertices.push_back(start);
		allVertices.push_back(end);

		if (allVertices.size())
		{
			DecoVertexBuffer vb;
			vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
			RI->SetColor(DecoColor(0xffff00ff));
			RI->SetLineWidth(2);
			RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
		}
	}

}

void ForceRecorder::drawExternalTorque(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	const double scale = 1e-1;
	vector<vector3> allVertices;
	vector3 totalForce = GetNetExternalTorque();
	vector3 start = mObj->GetCenterOfMass();
	vector3 end = start + scale * (totalForce);
	allVertices.push_back(start);
	allVertices.push_back(end);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	RI->SetColor(DecoColor(0xff00ff00));
	RI->SetLineWidth(2);
	RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
}
void ForceRecorder::drawInternalTorque(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	const double scale = 1e-1;
	vector<vector3> allVertices;
	vector3 totalForce = GetNetInternalTorque();
	vector3 start = mObj->GetCenterOfMass();
	vector3 end = start + scale * (totalForce);
	allVertices.push_back(start);
	allVertices.push_back(end);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	RI->SetColor(DecoColor(0xffff0000));
	RI->SetLineWidth(2);
	RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
}

void ForceRecorder::drawTotalTorque(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	const double scale = 1e-1;
	vector<vector3> allVertices;
	vector3 totalForce = GetNetContactTorque();

	vector3 start = mObj->GetCenterOfMass();
	vector3 end = start + scale * (totalForce);
	allVertices.push_back(start);
	allVertices.push_back(end);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	RI->SetColor(DecoColor(0xff0000ff));
	RI->SetLineWidth(2);
	RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
    //LOG(INFO) << "Net Torque is: " << totalForce;
}

void ForceRecorder::drawExternalForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	const double scale = 1e-1;
	vector<vector3> allVertices;
	vector3 totalForce = GetNetExternalForce();
	vector3 start = mObj->GetCenterOfMass();
	vector3 end = start + scale * (totalForce);
	allVertices.push_back(start);
	allVertices.push_back(end);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	RI->SetColor(DecoColor(0xffffffff));
	RI->SetLineWidth(2);
	RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
}
void ForceRecorder::drawInternalForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	const double scale = 1e-2;
	vector<vector3> allVertices;
	vector3 totalForce = GetNetInternalForce();
	vector3 start = mObj->GetCenterOfMass();
	vector3 end = start + scale * (totalForce);
	allVertices.push_back(start);
	allVertices.push_back(end);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	RI->SetColor(DecoColor(0xffffffff));
	RI->SetLineWidth(2);
	RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
}

void ForceRecorder::drawTotalForce(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType) const
{
	int numNodes = static_cast<int>(mObj->mNodes.size());
	const double scale = 1e-1;
	vector<vector3> allVertices;
	vector3 totalForce = GetNetContactForce();

	vector3 start = mObj->GetCenterOfMass();
	vector3 end = start + scale * (totalForce);
	allVertices.push_back(start);
	allVertices.push_back(end);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	RI->SetColor(DecoColor(0xffffffff));
	RI->SetLineWidth(2);
	RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);
}

DecoArchive& operator<< (DecoArchive& Ar, const ForceRecorder& force)
{
	if (force.mVersion >= 0.1)
	{
		Ar << force.mNormalContactForces;
		Ar << force.mFrictionContactForces; 
	}
	if (force.mVersion >= 0.2)
	{
		Ar << force.mTotalForces;
		Ar << force.mMuscleForce;
		Ar << force.mGravityForce;
		Ar << force.mElasticForce;
	}
	if (force.mVersion >= 0.1)
	{
		Ar << force.mNodeInContact;
	}
	if (force.mVersion >= 0.3)
	{
		Ar << force.mContactPolygon;
	}
	if (force.mVersion >= 0.4)
	{
		Ar << force.mPerturbationForce;
	}
	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, ForceRecorder& force)
{
	if (force.mVersion >= 0.1)
	{
		Ar >> force.mNormalContactForces;
		Ar >> force.mFrictionContactForces; 
	}
	if (force.mVersion >= 0.2)
	{
		Ar >> force.mTotalForces;
		Ar >> force.mMuscleForce;
		Ar >> force.mGravityForce;
		Ar >> force.mElasticForce;
	}
	if (force.mVersion >= 0.1)
	{
		Ar >> force.mNodeInContact;
	}
	if (force.mVersion >= 0.3)
	{
		Ar >> force.mContactPolygon;
	}
	if (force.mVersion >= 0.4)
	{
		Ar >> force.mPerturbationForce;
	}
	return Ar;
}