#include "ActiveSoftBody.h"
#include "utility/polarDecomposition.h"
#include "ForceRecorder.h"
#include "PlayBackFrame.h"
#include "controller/BaseAreaObjective.h"
#include "utility/EigenSerializer.h"

ActiveSoftBody::ActiveSoftBody() : mNumFiberSegments(0), mbDesiredMuscleLengthSet(false), mbMuscleActivationSet(false), mbHasLastFrame(false), mFootArea(0)
{
	if (mDeformedShape)
		delete mDeformedShape;

	mKFiber = 10 * mYoungsModulus;
	mDeformedShape = new ActiveSoftBodyRenderData();
	mDeformedShape->SetID(objID);
	mDeformedShape->SetObject(this);
	mContactPolygon.SetObj(this);
}

ActiveSoftBody::~ActiveSoftBody()
{

}

void ActiveSoftBody::SetYoungsModulus(double modulus)
{
	mYoungsModulus = modulus;
	double minYoungsModulos = 1e30;
	int numTets = mDeformedShape->GetNumTetrahedra();
	for (int i = 0; i < numTets; ++i)
	{
		if (minYoungsModulos >= mYoungsRatioPerTet[i])
			minYoungsModulos = mYoungsRatioPerTet[i];
	}
	mKFiber = 10 * minYoungsModulos * mYoungsModulus;
	precomputation();
}

BOOL ActiveSoftBody::Move(const vector3& offset)
{
    BOOL ret = DecoSoftObjectCorotationLinearFEM::Move(offset);

    int numFibers = static_cast<int>(mMuscleFibers.size());
    for (int i = 0; i < numFibers; ++i)
    {
        mMuscleFibers[i].SetCurrentAsRest();
    }
    return ret;
}
BOOL ActiveSoftBody::Rotate(AXIS axis, float rad, BOOL bAlongGlobal)
{
    BOOL ret = DecoSoftObjectCorotationLinearFEM::Rotate(axis, rad);
    int numFibers = static_cast<int>(mMuscleFibers.size());
    for (int i = 0; i < numFibers; ++i)
    {
        mMuscleFibers[i].SetCurrentAsRest();
    }
    return ret;
}
BOOL ActiveSoftBody::Scale(const vector3& fac)
{
    BOOL ret = DecoSoftObjectCorotationLinearFEM::Scale(fac);
    int numFibers = static_cast<int>(mMuscleFibers.size());
    for (int i = 0; i < numFibers; ++i)
    {
        mMuscleFibers[i].SetCurrentAsRest();
    }
    return ret;
}

DecoRenderData* ActiveSoftBody::GetRenderData()
{
	if (!mSelectedMuscleIds.empty())
	{
		int selectedMuscleId = mSelectedMuscleIds[0];
		int numVertices = static_cast<int>(mDeformedShape->GetNumVertices());
		vector<DecoColor> colors;
		colors.resize(numVertices);
		double maxWeight = 0;
		for (int i = 0; i < numVertices; ++i)
		{
			double weight = mPointMuscleWeights[i][selectedMuscleId];
			if (maxWeight < weight)
				maxWeight = weight;
		}
		for (int i = 0; i < numVertices; ++i)
		{
			double weight = mPointMuscleWeights[i][selectedMuscleId];
			double normalizedWeight = weight / maxWeight;
			CHECK(normalizedWeight <= 1.0 && normalizedWeight >= 0.0);
			colors[i] = DecoColor(vector4(1.0, 1.0 - normalizedWeight, 1.0 - normalizedWeight, 1.0));
		}
		mDeformedShape->SetPointColor(colors);
	}
	return DecoSoftObjectCorotationLinearFEM::GetRenderData();
}

void ActiveSoftBody::ReadMuscleFiberFromFile(const string& filename)
{
    ifstream in(filename.c_str());
    vector<MuscleDof> specialDofs;
    while (!in.eof())
    {
        MuscleFiber fiber;
        int ithFiber = static_cast<int>(mMuscleFibers.size());
        int groupId = fiber.ReadFromFile(in, this, mMuscleDofs, specialDofs, ithFiber);
        if (groupId == -1)
            continue;
        if (groupId >= static_cast<int>(mFiberGroups.size()))
        {
			int prevSize = static_cast<int>(mFiberGroups.size());

            //CHECK(groupId == mFiberGroups.size());
			mFiberGroups.resize(groupId + 1);
			for (int i = prevSize; i < groupId; ++i)
			{
				FiberGroup newGroup;
				mFiberGroups[groupId] = newGroup;
			}
			FiberGroup newGroup;
            newGroup.push_back(ithFiber);
            mFiberGroups[groupId] = newGroup;
        }
        else
        {
            mFiberGroups[groupId].push_back(ithFiber);
        }
        AddMuscleFiber(fiber);
    }
    mMuscleDofs.insert(mMuscleDofs.end(), specialDofs.begin(), specialDofs.end());
    BindAllMuscleFiber();

}

int ActiveSoftBody::GetNumMuscleDofs() const
{
	return static_cast<int>(mMuscleDofs.size());
}

int ActiveSoftBody::GetNumMuscleFibers() const
{
	return static_cast<int>(mMuscleFibers.size());
}

void ActiveSoftBody::AddMuscleFiber(MuscleFiber& fiber)
{
	fiber.SetSoftBody(this);
	mMuscleFibers.push_back(fiber);
}

void ActiveSoftBody::BindAllMuscleFiber()
{
	calculateMuscleCoordAndIndexInTet();
	calculateMuscleWeights();
	normalizeMuscleWeights();
	visualizeMuscleWeights();
	ActiveSoftBodyRenderData* activeRenderData = static_cast<ActiveSoftBodyRenderData*>(mDeformedShape);
	activeRenderData->SetMuscleFibers(&mMuscleFibers);
	mNumFiberSegments = countNumFiberSegments();

	constructActivationMapping();
	constructMuscleDofToActivationMatrixSmooth();

	bindMuscleDofToFiber();
}

void ActiveSoftBody::SetSelectMuscle(int selectedMuscleId, bool bAppend)
{
    if (!bAppend)
        mSelectedMuscleIds.clear();

    vector<int>::iterator it = find(mSelectedMuscleIds.begin(), mSelectedMuscleIds.end(), selectedMuscleId);
    if (it != mSelectedMuscleIds.end())
    {
        mSelectedMuscleIds.erase(it);
        LOG(INFO) << "Deselect muscle " << selectedMuscleId;
    }
    else
    {
        mSelectedMuscleIds.push_back(selectedMuscleId);
        LOG(INFO) << "Select muscle " << selectedMuscleId;
    }
    ActiveSoftBodyRenderData* activeRenderData = static_cast<ActiveSoftBodyRenderData*>(mDeformedShape);
    activeRenderData->SetSelectedMuscles(mSelectedMuscleIds);
}
bool ActiveSoftBody::SelectMuscles(const vector3& origin, const vector3& dir, double& rayDepth, int& muscleId)
{
    int numMuscles = static_cast<int>(mMuscleFibers.size());
    const double threshold = 0.1;
    double minDist = 1e20;
    for (int i = 0; i < numMuscles; ++i)
    {
        double depth;
        const MuscleFiber& fiber = mMuscleFibers[i];
        const vector<Vector3d>& curve = fiber.GetPositions();
        double dist = RayPolylineDistance(origin, dir, curve, depth);
        if (dist < minDist)
        {
            minDist = dist;
            rayDepth = depth;
            muscleId = i;
        }
    }
    if (minDist < threshold)
        return true;
    else
        return false;
}


void ActiveSoftBody::bindMuscleDofToFiber()
{
	int numDofs = static_cast<int>(mMuscleDofs.size());
	for (int i = 0; i < numDofs; ++i)
	{
		MuscleDof& dof = mMuscleDofs[i];
		const vector<int>& ithFiber = dof.GetFiberIndex();
        for (size_t fiberId = 0; fiberId < ithFiber.size(); ++fiberId)
		    mMuscleFibers[ithFiber[fiberId]].AddDof(&dof);
	}
}

void ActiveSoftBody::constructMuscleDofToActivationMatrix()
{
	int numMuscleDof = static_cast<int>(mMuscleDofs.size());
	mMuscleDofToActivation = MatrixXd::Zero(mNumFiberSegments, numMuscleDof);
	for (int i = 0; i < numMuscleDof; ++i)
	{
		MuscleDof& dof = mMuscleDofs[i];
        int numFiberControled = static_cast<int>(dof.mIthFiber.size());
        for (int ithFiber = 0; ithFiber < numFiberControled; ++ithFiber)
        {
		    for (int j = 0; j < dof.mNumSegmentInControl[ithFiber]; ++j)
		    {
			    int idx = mActivationMapping[dof.mIthFiber[ithFiber]][dof.mStartSegmentIdx[ithFiber] + j];
			    mMuscleDofToActivation(idx, i) = 1.0;
		    }
        }
	}
}

void ActiveSoftBody::constructMuscleDofToActivationMatrixSmooth()
{
	int numMuscleDof = static_cast<int>(mMuscleDofs.size());
	mMuscleDofToActivation = MatrixXd::Zero(mNumFiberSegments, numMuscleDof);
	for (int i = 0; i < numMuscleDof; ++i)
	{
		MuscleDof& dof = mMuscleDofs[i];
        int numFibersInDof = static_cast<int>(dof.mIthFiber.size());
        for (int ithFiber = 0; ithFiber < numFibersInDof; ++ithFiber)
        {
		    double dofSegmentsCenterIdx = (dof.mStartSegmentIdx[ithFiber] + dof.mStartSegmentIdx[ithFiber] + dof.mNumSegmentInControl[ithFiber] - 1) / 2.0;
		    int prevDofId = findPreviousDofInFiber(dof, ithFiber);
		    int nextDofId = findNextDofInFiber(dof, ithFiber);

		    for (int j = 0; j < dof.mNumSegmentInControl[ithFiber]; ++j)
		    {
			    int currentSegmentId = dof.mStartSegmentIdx[ithFiber] + j;
			    int idx = mActivationMapping[dof.mIthFiber[ithFiber]][currentSegmentId];
			    if (prevDofId == -1 && nextDofId == -1)
			    {
				    mMuscleDofToActivation(idx, i) = 1.0;
			    }
			    else if (abs(currentSegmentId - dofSegmentsCenterIdx) < 1e-6)
			    {
				    mMuscleDofToActivation(idx, i) = 1.0;
			    }
			    else if (currentSegmentId < dofSegmentsCenterIdx)
			    {
    				
				    if (prevDofId != -1)
				    {
					    MuscleDof& prevDof = mMuscleDofs[prevDofId];
					    double prevDofCenterSegmentIdx = (prevDof.mStartSegmentIdx[ithFiber] + prevDof.mStartSegmentIdx[ithFiber] + prevDof.mNumSegmentInControl[ithFiber] - 1) / 2.0;

					    mMuscleDofToActivation(idx, i) = static_cast<double>(currentSegmentId - prevDofCenterSegmentIdx) / (dofSegmentsCenterIdx - prevDofCenterSegmentIdx);
					    mMuscleDofToActivation(idx, prevDofId) = 1.0 - mMuscleDofToActivation(idx, i);
				    }
				    else
				    {
					    CHECK(nextDofId != -1);
					    MuscleDof& nextDof = mMuscleDofs[nextDofId];
					    double nextDofCenterSegmentIdx = (nextDof.mStartSegmentIdx[ithFiber] + nextDof.mStartSegmentIdx[ithFiber] + nextDof.mNumSegmentInControl[ithFiber] - 1) / 2.0;
					    mMuscleDofToActivation(idx, nextDofId) = static_cast<double>(currentSegmentId - dofSegmentsCenterIdx) / (nextDofCenterSegmentIdx - dofSegmentsCenterIdx);
					    mMuscleDofToActivation(idx, i) = 1.0 - mMuscleDofToActivation(idx, nextDofId);
				    }
			    }
			    else
			    {
				    if (nextDofId != -1)
				    {
					    MuscleDof& nextDof = mMuscleDofs[nextDofId];
					    double nextDofCenterSegmentIdx = (nextDof.mStartSegmentIdx[ithFiber] + nextDof.mStartSegmentIdx[ithFiber] + nextDof.mNumSegmentInControl[ithFiber] - 1) / 2.0;
					    mMuscleDofToActivation(idx, nextDofId) = static_cast<double>(currentSegmentId - dofSegmentsCenterIdx) / (nextDofCenterSegmentIdx - dofSegmentsCenterIdx);
					    mMuscleDofToActivation(idx, i) = 1.0 - mMuscleDofToActivation(idx, nextDofId);
				    }
				    else
				    {
					    CHECK(prevDofId != -1);
					    MuscleDof& prevDof = mMuscleDofs[prevDofId];
					    double prevDofCenterSegmentIdx = (prevDof.mStartSegmentIdx[ithFiber] + prevDof.mStartSegmentIdx[ithFiber] + prevDof.mNumSegmentInControl[ithFiber] - 1) / 2.0;
					    mMuscleDofToActivation(idx, i) = static_cast<double>(currentSegmentId - prevDofCenterSegmentIdx) / (dofSegmentsCenterIdx - prevDofCenterSegmentIdx);
					    mMuscleDofToActivation(idx, prevDofId) = 1.0 - mMuscleDofToActivation(idx, i);

				    }
			    }
            }
		}
	}
}

int ActiveSoftBody::findPreviousDofInFiber(const MuscleDof& currentDof, int fiberIndex)
{
	int ret = -1;
	int numDofs = static_cast<int>(mMuscleDofs.size());
	for (int i = 0; i < numDofs; ++i)
	{
		if (&currentDof == &mMuscleDofs[i]) continue;
		MuscleDof& dofToInvestigate = mMuscleDofs[i];
        if (dofToInvestigate.mIthFiber.size() <= fiberIndex) continue;

		if (dofToInvestigate.mIthFiber[fiberIndex] != currentDof.mIthFiber[fiberIndex]) continue;
		if (dofToInvestigate.mStartSegmentIdx[fiberIndex] + dofToInvestigate.mNumSegmentInControl[fiberIndex] == currentDof.mStartSegmentIdx[fiberIndex])
		{
			ret = i;
			break;
		}
	}
	return ret;
}
int ActiveSoftBody::findNextDofInFiber(const MuscleDof& currentDof, int fiberIndex)
{
	int ret = -1;
	int numDofs = static_cast<int>(mMuscleDofs.size());
	for (int i = 0; i < numDofs; ++i)
	{
		if (&currentDof == &mMuscleDofs[i]) continue;
		MuscleDof& dofToInvestigate = mMuscleDofs[i];
        if (dofToInvestigate.mIthFiber.size() <= fiberIndex) continue;

		if (dofToInvestigate.mIthFiber[fiberIndex] != currentDof.mIthFiber[fiberIndex]) continue;
		if (dofToInvestigate.mStartSegmentIdx[fiberIndex] == currentDof.mStartSegmentIdx[fiberIndex] + currentDof.mNumSegmentInControl[fiberIndex])
		{
			ret = i;
			break;
		}
	}
	return ret;
}

void ActiveSoftBody::constructActivationMapping()
{
	int numMuscleFibers = static_cast<int>(mMuscleFibers.size());
	mActivationMapping.resize(numMuscleFibers);
	int segmentCount = 0;
	for (int i = 0; i < numMuscleFibers; ++i)
	{
		int numSegments = mMuscleFibers[i].GetNumSegments();
		mActivationMapping[i].resize(numSegments);
		for (int ithSegment = 0; ithSegment < numSegments; ++ithSegment)
		{
			mActivationMapping[i][ithSegment] = segmentCount++;
		}
	}
}

void ActiveSoftBody::SetMuscleDofValues(const VectorXd& muscleDofValues)
{
	mbMuscleActivationSet = true;
	int numMuscleDofs = static_cast<int>(mMuscleDofs.size());
	for (int i = 0; i < numMuscleDofs; ++i)
	{
		mMuscleDofs[i].mValue = muscleDofValues[i];
	}
	calculateControlForce();
	exertControlForce();
	
}

void ActiveSoftBody::SetFootVertexIdx(const vector<int>& footVId)
{
	mFootVertexIdx = footVId;
	mFootArea = BaseAreaObjective::CalcualteProjectedBaseArea(this, mFootVertexIdx, vector3(0, 1, 0));
}

const vector<int>& ActiveSoftBody::GetFootVertexId() const
{
	return mFootVertexIdx;
}

double ActiveSoftBody::GetFootArea() const
{
	return mFootArea;
}

Vector3d ActiveSoftBody::GetVelFootCenter() const
{
    int numFootIds = static_cast<int>(mFootVertexIdx.size());
    vector3 center(0, 0, 0);
    for (int i = 0; i < numFootIds; ++i)
    {
        center += mNodes[mFootVertexIdx[i]].mVel;
    }
    center /= numFootIds;
    return Vector3d(center.x, center.y, center.z);
}

Vector3d ActiveSoftBody::GetFootCenter() const
{
    int numFootIds = static_cast<int>(mFootVertexIdx.size());
    vector3 center(0, 0, 0);
    for (int i = 0; i < numFootIds; ++i)
    {
        center += mNodes[mFootVertexIdx[i]].mPos;
    }
    center /= numFootIds;
    return Vector3d(center.x, center.y, center.z);
}

void ActiveSoftBody::UpdateContactPolygon(const vector<int>& contactNodeIndices)
{
	vector<vector3> nodesInContact;

	int numNodesInContact = static_cast<int>(contactNodeIndices.size());
	mContactVertices.clear();
	for (int i = 0; i < numNodesInContact; ++i)
	{
		vector3 pos = mNodes[contactNodeIndices[i]].mPos;
		//		pos.y = projectHeight;
		mContactVertices.push_back(contactNodeIndices[i]);
	}

	if (!mFootVertexIdx.empty())
	{
		int numFootVertices = static_cast<int>(mFootVertexIdx.size());
		for (int i = 0; i < numFootVertices; ++i)
		{
			vector3 pos = mNodes[mFootVertexIdx[i]].mPos;
			nodesInContact.push_back(pos);
		}
	}
	else
	{
		for (int i = 0; i < numNodesInContact; ++i)
		{
			vector3 pos = mNodes[contactNodeIndices[i]].mPos;
			nodesInContact.push_back(pos);
		}
	}

	mContactPolygon.SetContactPoints(nodesInContact);
	const vector<vector3>& realContactVertices = mContactPolygon.GetVertices();
	mForces->mContactPolygon = realContactVertices;
}

void ActiveSoftBody::Update(double timeStep)
{
	DecoSoftObjectCorotationLinearFEM::Update(timeStep);
	UpdateMusclePosition();
    UpdateBoundingBox();
}

void ActiveSoftBody::UpdateBoundingBox()
{
    int numNodes = GetNumNodes();

    vector<vector3> vertices;
    vertices.resize(numNodes);

    for (int i = 0; i < numNodes; ++i)
    {
        vector3 pos = GetIthVertex(i);
        vertices[i] = pos;
    }
    mbb = Bounding(&(vertices[0]), numNodes);
}

int ActiveSoftBody::countNumFiberSegments()
{
	int numSegments = 0;
	int numFibers = static_cast<int>(mMuscleFibers.size());
	for (int i = 0; i < numFibers; ++i)
	{
		numSegments += mMuscleFibers[i].GetNumSegments();
	}
	return numSegments;
}
void ActiveSoftBody::calculateMuscleWeights()
{
	int numTet = mRestShape->GetNumTetrahedra();
	mMuscleWeights.resize(numTet);

	int numMuscleFibers = static_cast<int>(mMuscleFibers.size());
	
	for (int ithTet = 0; ithTet < numTet; ++ithTet)
	{
		Tetrahedron tet = mRestShape->GetIthTetrahedron(ithTet);
		mMuscleWeights[ithTet].resize(numMuscleFibers);
		for (int ithFiber = 0; ithFiber < numMuscleFibers; ++ithFiber)
		{
			int numSegments = mMuscleFibers[ithFiber].GetNumSegments();
			mMuscleWeights[ithTet][ithFiber].resize(numSegments);
			for (int ithSegment = 0; ithSegment < numSegments; ++ithSegment)
			{
				mMuscleWeights[ithTet][ithFiber][ithSegment] = 0.0;
			}
			int ithSegment = -1;
			const vector<Vector3d>& polyline = mMuscleFibers[ithFiber].GetPolylineRepresentationRestShape();
			vector<vector3> polylineRightFormat;
			polylineRightFormat.resize(polyline.size());
			for (size_t ithVertex = 0; ithVertex < polyline.size(); ++ithVertex)
			{
				polylineRightFormat[ithVertex] = vector3(polyline[ithVertex](0), polyline[ithVertex](1), polyline[ithVertex](2));
			}
			double dist = PointPolylineDistance(tet.CalculateCenter(), polylineRightFormat, ithSegment);
			double weight = calculateFromKernel(dist, mMuscleFibers[ithFiber].GetInfluenceSigma());
			mMuscleWeights[ithTet][ithFiber][ithSegment] = weight;
		}
	}
}

void ActiveSoftBody::visualizeMuscleWeights()
{
	int numTet = mRestShape->GetNumTetrahedra();
	int numVertices = mRestShape->GetNumVertices();
	int numFibers = static_cast<int>(mMuscleFibers.size());
	mPointMuscleWeights.resize(numVertices);
	for (int i = 0; i < numVertices; ++i)
	{
		mPointMuscleWeights[i].insert(mPointMuscleWeights[i].end(), numFibers, 0);
	}
	for (int ithTet = 0; ithTet < numTet; ++ithTet)
	{
		Tetrahedron tet = mRestShape->GetIthTetrahedron(ithTet);
		int a = tet.mVertIndices[0];
		int b = tet.mVertIndices[1];
		int c = tet.mVertIndices[2];
		int d = tet.mVertIndices[3];
		for (int jthFiber = 0; jthFiber < numFibers; ++jthFiber)
		{
			int numSegments = static_cast<int>(mMuscleWeights[ithTet][jthFiber].size());
			for (int kthSegment = 0; kthSegment < numSegments; ++kthSegment)
			{
				double weight = mMuscleWeights[ithTet][jthFiber][kthSegment];
				for (int ithVert = 0; ithVert < 4; ++ithVert)
				{
					mPointMuscleWeights[tet.mVertIndices[ithVert]][jthFiber] += weight;
				}
			}
		}
	}
}
void ActiveSoftBody::normalizeMuscleWeights()
{
	int numTet = mRestShape->GetNumTetrahedra();

	int numFibers = static_cast<int>(mMuscleFibers.size());
	int numFiberGroups = static_cast<int>(mFiberGroups.size());

	for (int ithTet = 0; ithTet < numTet; ++ithTet)
	{
		for (int ithGroup = 0; ithGroup < numFiberGroups; ++ithGroup)
		{
			double totalWeight = 0;
			int numFibersInGroup = static_cast<int>(mFiberGroups[ithGroup].size());
			const FiberGroup& group = mFiberGroups[ithGroup];
			for (int ithFiber = 0; ithFiber < numFibersInGroup; ++ithFiber)
			{
				int fiberId = group[ithFiber];
				int numSegments = static_cast<int>(mMuscleFibers[fiberId].GetNumSegments());
				for (int ithSegment = 0; ithSegment < numSegments; ++ithSegment)
				{
					totalWeight += mMuscleWeights[ithTet][fiberId][ithSegment];
				}
			}
			for (int ithFiber = 0; ithFiber < numFibersInGroup; ++ithFiber)
			{
				int fiberId = group[ithFiber];
				int numSegments = static_cast<int>(mMuscleFibers[fiberId].GetNumSegments());
				for (int ithSegment = 0; ithSegment < numSegments; ++ithSegment)
				{
                    if (totalWeight == 0.0)
                        mMuscleWeights[ithTet][fiberId][ithSegment] = 0.0;
                    else
					    mMuscleWeights[ithTet][fiberId][ithSegment] /= totalWeight;
				}
			}
		}
	}
}

void ActiveSoftBody::exertControlForce()
{
	int numNodes = static_cast<int>(mNodes.size());
	for (int i = 0; i < numNodes; ++i)
	{
		ExertForceOnNode(i, vector3(mControlForces[i](0), mControlForces[i](1), mControlForces[i](2)));
		mForces->mMuscleForce[i] = vector3(mControlForces[i](0), mControlForces[i](1), mControlForces[i](2));
	}
}

void ActiveSoftBody::UpdateMusclePosition()
{
	int numMuscles = static_cast<int>(mMuscleFibers.size());
	for (int i = 0; i < numMuscles; ++i)
	{
		mMuscleFibers[i].UpdatePosition();
	}
	
}

double ActiveSoftBody::calculateFromKernel(double dist, double sigma)
{
	
	double weight = exp(-dist * dist / (2 * sigma * sigma));
	return weight;
}

void ActiveSoftBody::calculateMuscleCoordAndIndexInTet()
{
	int numMuscles = static_cast<int>(mMuscleFibers.size());
	for (int i = 0; i < numMuscles; ++i)
	{
		mMuscleFibers[i].UpdateBarycentricCoordAndTetIndex();
	}
}

bool ActiveSoftBody::GetDesiredMuscleLengthRelativeToRestLastFrame(VectorXd& desiredMuscleLength) const
{
	if (mDesiredMuscleLengthLastFrame.size())
	{
		desiredMuscleLength = mDesiredMuscleLengthLastFrame;
		return true;
	}
	return false;
}

void ActiveSoftBody::GetDesiredMuscleLengthRelativeToRest(VectorXd& desiredMuscleLength) const
{
	desiredMuscleLength = mDesiredMuscleLength;
}

void ActiveSoftBody::GetCurrentMuscleLengthRelativeToRest(VectorXd& currentMuscleLength) const
{
	int numMuscleDofs = static_cast<int>(mMuscleDofs.size());
	currentMuscleLength = VectorXd::Zero(numMuscleDofs);
	for (int i = 0; i < numMuscleDofs; ++i)
	{
		currentMuscleLength[i] = mMuscleDofs[i].GetCurrentLengthRelativeToRest();
	}
}

void ActiveSoftBody::SetDesiredMuscleLengthRelativeToRest()
{
	if (mDesiredMuscleLength.size() != 0)
	{
		VectorXd dofValues = TransformFromMuscleLengthToDof(mDesiredMuscleLength);
		SetMuscleDofValues(dofValues);
	}
}

void ActiveSoftBody::SetDesiredMuscleLengthRelativeToRest(const VectorXd& desiredMuscleLength)
{
	mDesiredMuscleLengthLastFrame = mDesiredMuscleLength;
	mDesiredMuscleLength = desiredMuscleLength;

	VectorXd dofValues = TransformFromMuscleLengthToDof(mDesiredMuscleLength);
	SetMuscleDofValues(dofValues);


}

void ActiveSoftBody::CalculateIntermediateVariable(double timeStep)
{
    int numNodeDofs = GetNumNodes() * 3;
    int numMuscleDofs = GetNumMuscleDofs();
    CalculateDeriv(timeStep, NULL); //deriv not used, just to let dfdx, force, etc. to be calculated.

    int numTotalDofs = numMuscleDofs;

    MatrixXd controlForceMatrix = MatrixXd::Zero(numNodeDofs, numTotalDofs);

    MatrixXd muscleMatrix = GetControlForceMatrix();

    controlForceMatrix = muscleMatrix;

    BlockSparseMatrix augmentedMass = GetAugmentedMassMatrix(timeStep);
    augmentedMass.ConjugateGradientSolve(controlForceMatrix, mIntermediate);


    BlockSparseMatrix mass = GetMassMatrix();

    VectorXd externalForce;
    GetForce(externalForce);
    VectorXd currentVel;
    GetVel(currentVel);

    augmentedMass.ConjugateGradientSolve(timeStep * externalForce + mass * currentVel, mCommonRhs);
    mCommonRhs /= timeStep;
}

void ActiveSoftBody::CalculateIntermediateVariable(double timeStep, const MatrixXd& contactNormals, const MatrixXd& contactTangents, const MatrixXd& anisotropicFrictionCoeff)
{
	int numNodeDofs = GetNumNodes() * 3;
	int numMuscleDofs = GetNumMuscleDofs();
	int numContactNormalDofs = contactNormals.cols();
	int numContactTangentDofs = contactTangents.cols();
	int numTotalDofs = numMuscleDofs + numContactNormalDofs + numContactTangentDofs;

	MatrixXd controlForceMatrix = MatrixXd::Zero(numNodeDofs, numTotalDofs);

	MatrixXd muscleMatrix = GetControlForceMatrix();
	double invTimeStep = 1.0 / timeStep;

	controlForceMatrix.block(0, 0, numNodeDofs, numMuscleDofs) = muscleMatrix;
	controlForceMatrix.block(0, numMuscleDofs, numNodeDofs, numContactNormalDofs) = invTimeStep * contactNormals;
	controlForceMatrix.block(0, numMuscleDofs + numContactNormalDofs, numNodeDofs, numContactTangentDofs) = invTimeStep * contactTangents * anisotropicFrictionCoeff;

	BlockSparseMatrix augmentedMass = GetAugmentedMassMatrix(timeStep);
	augmentedMass.ConjugateGradientSolve(controlForceMatrix, mIntermediate);

	CalculateDeriv(timeStep, NULL); //deriv not used, just to let dfdx, force, etc. to be calculated.

	BlockSparseMatrix mass = GetMassMatrix();

	//VectorXd elasticForce = CalculateElasticForce(timeStep);
	//VectorXd gravityForce = CalculateGravityForce(timeStep);
	//VectorXd externalForce = elasticForce + gravityForce;
    VectorXd externalForce;
    GetForce(externalForce);
	VectorXd currentVel;
	GetVel(currentVel);

	augmentedMass.ConjugateGradientSolve(timeStep * externalForce + mass * currentVel, mCommonRhs);
	mCommonRhs /= timeStep;
}


 
MuscleDof& ActiveSoftBody::GetMuscleDof(int ithDof)
{
	return mMuscleDofs[ithDof];
}

MuscleFiber& ActiveSoftBody::GetMuscleFiber(int ithFiber)
{
	return mMuscleFibers[ithFiber];
}

VectorXd ActiveSoftBody::TransformFromMuscleLengthToDof(const VectorXd& muscleLength)
{
	int numMuscleDofs = static_cast<int>(mMuscleDofs.size());
	VectorXd muscleActivation = VectorXd::Zero(numMuscleDofs);
	for (int i = 0; i < numMuscleDofs; ++i)
	{
		double currentLength = mMuscleDofs[i].GetCurrentLength();
		double restLength = mMuscleDofs[i].GetRestLength();
		double desiredLength = muscleLength[i] * restLength;
		double activation = mKFiber * (currentLength - desiredLength) / desiredLength;
		muscleActivation[i] = activation;

	}
	return muscleActivation;
}

VectorXd ActiveSoftBody::TransformFromDofToMuscleLength(const VectorXd& muscleDof)
{
	int numMuscleDofs = static_cast<int>(mMuscleDofs.size());
	VectorXd muscleLength = VectorXd::Zero(numMuscleDofs);

	for (int i = 0; i < numMuscleDofs; ++i)
	{
		double currentLength = mMuscleDofs[i].GetCurrentLength();
		double restLength = mMuscleDofs[i].GetRestLength();
		double desiredLength = mKFiber * currentLength / (mKFiber + muscleDof[i]) / restLength;
		muscleLength[i] = desiredLength;

	}
	return muscleLength;
}


const MatrixXd& ActiveSoftBody::GetControlForceMatrix()
{
	calculateControlForceMatrix();
	return mMuscleDofToControlForce;
}

const MatrixXd& ActiveSoftBody::GetMuscleDofActivationMatrix() const
{
	return mMuscleDofToActivation;
}

Box ActiveSoftBody::GetBoundingBox() const
{
    return mbb;
}


void ActiveSoftBody::calculateControlForceMatrix()
{
	int numVertices = static_cast<int>(mNodes.size());
	mMuscleDofToControlForce = MatrixXd::Zero(3 * numVertices, mNumFiberSegments);

	int numTets = mDeformedShape->GetNumTetrahedra();
	for (int ithTet = 0; ithTet < numTets; ++ithTet)
	{
		Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
		matrix33 P;
		P[0] = tet.mVertices[1] - tet.mVertices[0];
		P[1] = tet.mVertices[2] - tet.mVertices[0];
		P[2] = tet.mVertices[3] - tet.mVertices[0];
		P = P * mRestShapeInvertedX[ithTet];

		Matrix3d R, S;
		Matrix3d transformation, lastTransformation;
		transformation << P[0][0], P[1][0], P[2][0],
			P[0][1], P[1][1], P[2][1],
			P[0][2], P[1][2], P[2][2];
		PolarDecomposition::DoPolarDecomposition(transformation, R, S);

		int numFibers = static_cast<int>(mMuscleFibers.size());
		int segmentCount = 0;
		for (int ithFiber = 0; ithFiber < numFibers; ++ithFiber)
		{
			int numSegments = static_cast<int>(mMuscleFibers[ithFiber].GetNumSegments());

			for (int ithSegment = 0; ithSegment < numSegments; ++ithSegment)
			{
				Matrix3d fiberR = mMuscleFibers[ithFiber].GetRotationFromXAxis(ithSegment);
				Matrix3d localToGlobal = R * fiberR;
				//Vector3d firstColumn = localToGlobal.col(0);
				Matrix3d magnitudeToLocalStress = Matrix3d::Zero();
				magnitudeToLocalStress(0, 0) = 1.0;// - mPoissonRatio;
				//magnitudeToLocalStress(1, 1) = -mPoissonRatio;
				//magnitudeToLocalStress(2, 2) = -mPoissonRatio;
				Matrix3d magnitudeToStress = localToGlobal * magnitudeToLocalStress * localToGlobal.transpose();
				//magnitudeToStress << firstColumn(0) * firstColumn(0), firstColumn(0) * firstColumn(1), firstColumn(0) * firstColumn(2), 
				//					 firstColumn(1) * firstColumn(0), firstColumn(1) * firstColumn(1), firstColumn(1) * firstColumn(2), 
				//					 firstColumn(2) * firstColumn(0), firstColumn(2) * firstColumn(1), firstColumn(2) * firstColumn(2); 
				for (int ithFace = 0; ithFace < 4; ++ithFace)
				{	
					Vector3d fFaceFromMagnitude = fromStressToForce(magnitudeToStress, tet, ithFace);
					int nodeIdx1 = tet.mVertIndices[ithFace];
					int nodeIdx2 = tet.mVertIndices[(ithFace + 1) % 4];
					int nodeIdx3 = tet.mVertIndices[(ithFace + 2) % 4];

					Vector3d fNodeFromMagnitude = fFaceFromMagnitude / 3.0 * mMuscleWeights[ithTet][ithFiber][ithSegment];
					mMuscleDofToControlForce.block<3, 1>(nodeIdx1 * 3, segmentCount) += fNodeFromMagnitude;
					mMuscleDofToControlForce.block<3, 1>(nodeIdx2 * 3, segmentCount) += fNodeFromMagnitude;
					mMuscleDofToControlForce.block<3, 1>(nodeIdx3 * 3, segmentCount) += fNodeFromMagnitude;
				}
				segmentCount++;
			}
		}
	}
    //EigenSerializer::SaveMatrixToFileDouble("dofToActivation.txt", mMuscleDofToActivation, false);
	mMuscleDofToControlForce *= mMuscleDofToActivation;
}


void ActiveSoftBody::calculateControlForce()
{
	int numNodes = static_cast<int>(mNodes.size());
	
	mControlForces.resize(numNodes);
	for (int i = 0; i < numNodes; ++i)
	{
		mControlForces[i] = Vector3d::Zero();
	}

	calculateControlForceMatrix();
	
	VectorXd muscleDof(mMuscleDofs.size());
	for (size_t i = 0; i < mMuscleDofs.size(); ++i)
	{
		muscleDof[i] = mMuscleDofs[i].mValue;
	}
	VectorXd muscleForce = mMuscleDofToControlForce * muscleDof;
	for (int i = 0; i < numNodes; ++i)
	{
		mControlForces[i] = muscleForce.segment<3>(3 * i);
	} 

}

void ActiveSoftBody::Serialize(DecoArchive& Ar) const
{
	DecoSoftObjectCorotationLinearFEM::Serialize(Ar);
	Ar << mMuscleFibers;
	Ar << mFiberGroups;
	Ar << mMuscleDofs;
	Ar << mKFiber;
	Ar << mNumFiberSegments;
}
void ActiveSoftBody::Deserialize(DecoArchive& Ar)
{
	DecoSoftObjectCorotationLinearFEM::Deserialize(Ar);
	vector<MuscleFiber> fibers;
	Ar >> fibers;
	for (size_t i = 0; i < fibers.size(); ++i)
		AddMuscleFiber(fibers[i]);
	Ar >> mFiberGroups;
	Ar >> mMuscleDofs;
	for (size_t i = 0; i < mMuscleDofs.size(); ++i)
		mMuscleDofs[i].SetSoftBody(this);
	Ar >> mKFiber;
	Ar >> mNumFiberSegments;
	BindAllMuscleFiber();
}

void ActiveSoftBody::ConstructTestBody(double timeStep, const string& shapeFilename, const string& muscleFileName)
{
    CreateSceneObjectFromMesh(shapeFilename, timeStep);

    mMuscleDofs.clear();
    ReadMuscleFiberFromFile(muscleFileName);
}

void ActiveSoftBody::ConstructTestBody(double timeStep, const string& filename)
{
	CreateSceneObjectFromMesh(filename, timeStep);

	mMuscleDofs.clear();

	MuscleFiber fiber1, fiber2, fiber3, fiber4, fiber5, fiber6, fiber7;
	//fiber1.ConstructTestLongitudinalFiber(this, mMuscleDofs, 0);
	//fiber1.Translate(Vector3d(0, -0.5, 0.0));
 //   //fiber1.Translate(Vector3d(0.0, 0.51, -1.0));
	//fiber2.ConstructTestLongitudinalFiber(this, mMuscleDofs, 1);
	//fiber2.Translate(Vector3d(0, 0.5, 0.0));
    //fiber2.Translate(Vector3d(0.0, 0.51, 1.0));
	fiber3.ConstructTestRadialHorizontalFiber(this, mMuscleDofs, 0);
	fiber4.ConstructTestRadialVerticalFiber(this, mMuscleDofs, 1);
	//fiber5.ConstructTestLongitudinalFiber(this, mMuscleDofs, 4);
	//fiber5.Translate(Vector3d(0, 0, -1.0));
 //   //fiber5.Translate(Vector3d(0.0, -0.5, -1.0));
	//fiber6.ConstructTestLongitudinalFiber(this, mMuscleDofs, 5);
	//fiber6.Translate(Vector3d(0, 0, 1.0));
    //fiber6.Translate(Vector3d(0, -0.5, 1.0));
	//fiber7.ConstructTestTwistFiber(this, mMuscleDofs, 0);

	//AddMuscleFiber(fiber1);
	//AddMuscleFiber(fiber2);
	AddMuscleFiber(fiber3);
	AddMuscleFiber(fiber4);
	//AddMuscleFiber(fiber5);
	//AddMuscleFiber(fiber6);
	//AddMuscleFiber(fiber7);




	FiberGroup group1, group2, group3, group4;
	//group1.push_back(0);
	//group1.push_back(1);
	//group3.push_back(4);
	//group3.push_back(5);
	group2.push_back(0);
	group2.push_back(1);
	//group4.push_back(0);
	
	//mFiberGroups.push_back(group1);
	mFiberGroups.push_back(group2);
	//mFiberGroups.push_back(group3);
	//mFiberGroups.push_back(group4);
	

	BindAllMuscleFiber();
}

void ActiveSoftBody::ConstructTestTargetBody(double timeStep, const string& filename)
{
	CreateSceneObjectFromMesh(filename, timeStep);
	mDeformedShape->SetColor(0xffff00ff);
	bIsUpdatable = FALSE;
}

void ActiveSoftBody::RecordMovie(DecoArchive& Ar, double playerVersion) const
{
	DecoSoftObject::RecordMovie(Ar, playerVersion);

	if (playerVersion >= 0.5)
	{
		Ar << mMuscleFibers;
	}
}
void ActiveSoftBody::PlaybackMovie(int ithFrame)
{
	DecoSoftObject::PlaybackMovie(ithFrame);
	int numCachedFrames = static_cast<int>(mPlaybackMovie.size());
	if (numCachedFrames <= ithFrame)
	{
		LOG(WARNING) << ithFrame << "th frame of " << name << " is not cached. Playback failed.";
		return;
	}
	const vector<MuscleFiber>& fibers = mPlaybackMovie[ithFrame].mMuscleFibers;
	mMuscleFibers = fibers;	
	ActiveSoftBodyRenderData* activeRenderData = static_cast<ActiveSoftBodyRenderData*>(mDeformedShape);
	activeRenderData->SetMuscleFibers(&mMuscleFibers);
}

