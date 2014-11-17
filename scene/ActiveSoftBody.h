#ifndef _ACTIVE_SOFT_BODY_H
#define _ACTIVE_SOFT_BODY_H

#include "MuscleFiber.h"
#include "MuscleDof.h"
#include "DecoSoftBodyCorotationLinearFEM.h"
#include "ActiveSoftBodyRenderData.h"
#include "utility/archive.h"
#include <vector>

using namespace std;

class ActiveSoftBody : public DecoSoftObjectCorotationLinearFEM
{
	friend class MuscleLengthPlanner;
public:
	ActiveSoftBody();
	virtual ~ActiveSoftBody();
	virtual void Update(double timeStep);
	virtual void SetYoungsModulus(double modulus);
	virtual DecoRenderData* GetRenderData();
    virtual BOOL Move(const vector3& offset);
    virtual BOOL Rotate(AXIS axis, float rad, BOOL bAlongGlobal = TRUE);
    virtual BOOL Scale(const vector3& fac);
	virtual void UpdateContactPolygon(const vector<int>& contactNodeIndices);

    void ReadMuscleFiberFromFile(const string& filename);
	void AddMuscleFiber(MuscleFiber& fiber);
	void BindAllMuscleFiber();
	void UpdateMusclePosition();
    void UpdateBoundingBox();
	void SetMuscleDofValues(const VectorXd& muscleDofValues);
	void SetFootVertexIdx(const vector<int>& footVId);
	const vector<int>& GetFootVertexId() const;
    Vector3d GetFootCenter() const;
    Vector3d GetVelFootCenter() const;
	double GetFootArea() const;

	const MatrixXd& GetControlForceMatrix();
	const MatrixXd& GetMuscleDofActivationMatrix() const;
	void SetDesiredMuscleLengthRelativeToRest(const VectorXd& desiredMuscleLength);
	void SetDesiredMuscleLengthRelativeToRest();

	int GetNumMuscleDofs() const;
	int GetNumMuscleFibers() const;
	void GetDesiredMuscleLengthRelativeToRest(VectorXd& desiredMuscleLength) const;
	void GetCurrentMuscleLengthRelativeToRest(VectorXd& currentMuscleLength) const;
	bool GetDesiredMuscleLengthRelativeToRestLastFrame(VectorXd& desiredMuscleLength) const;
	MuscleFiber& GetMuscleFiber(int ithFiber);
	MuscleDof& GetMuscleDof(int ithDof);
    void SetSelectMuscle(int selectedMuscleId, bool bAppend);
    bool SelectMuscles(const vector3& origin, const vector3& dir, double& rayDepth, int& muscleId);

	VectorXd TransformFromDofToMuscleLength(const VectorXd& muscleDof);
	VectorXd TransformFromMuscleLengthToDof(const VectorXd& muscleLength);
	void CalculateIntermediateVariable(double timeStep, const MatrixXd& contactNormals, const MatrixXd& contactTangents, const MatrixXd& anisotropicFrictionCoeff); //used for controllers
    void CalculateIntermediateVariable(double timeStep); //used for controllers

    Box GetBoundingBox() const;
	void ConstructTestBody(double timeStep, const string& filename);
    void ConstructTestBody(double timeStep, const string& shapeFilename, const string& muscleFileName);
	void ConstructTestTargetBody(double timeStep, const string& filename);
    virtual void RecordMovie(DecoArchive& Ar, double playerVersion) const;
	virtual void PlaybackMovie(int ithFrame);
	virtual void Serialize(DecoArchive& Ar) const;
	virtual void Deserialize(DecoArchive& Ar);

	VectorXd mCommonRhs;
	MatrixXd mIntermediate;

protected:
	void calculateMuscleWeights();
	void normalizeMuscleWeights();
	void visualizeMuscleWeights();
	void calculateMuscleCoordAndIndexInTet();
	void exertControlForce();
	void constructActivationMapping();
	void constructMuscleDofToActivationMatrix();
	void constructMuscleDofToActivationMatrixSmooth();
	int findPreviousDofInFiber(const MuscleDof& currentDof, int fiberIndex);
	int findNextDofInFiber(const MuscleDof& currentDof, int fiberIndex);

	void calculateControlForceMatrix(); //calculate the transformation from the muscleLength to the control force
	void calculateControlForce();
	double calculateFromKernel(double dist, double sigma);
	int countNumFiberSegments();
	void bindMuscleDofToFiber();

	typedef vector<int> FiberGroup;
	vector<FiberGroup> mFiberGroups;
	vector<MuscleDof> mMuscleDofs;
	vector<MuscleFiber> mMuscleFibers;
	vector<Vector3d> mControlForces;
	vector<int> mFootVertexIdx;
	ActiveSoftBodyRenderData mRenderData;
	vector<vector<vector<double> > > mMuscleWeights; 
	vector<vector<double> > mPointMuscleWeights; //for visualization only
	vector<vector<int> > mActivationMapping;
	MatrixXd mMuscleDofToControlForce;
	MatrixXd mMuscleDofToActivation;
	double mKFiber;
	int mNumFiberSegments;
	bool mbDesiredMuscleLengthSet;
	bool mbMuscleActivationSet;
	VectorXd mDesiredMuscleLength;
	VectorXd mDesiredMuscleLengthLastFrame;
	bool mbHasLastFrame;
	double mFootArea;
    vector<int> mSelectedMuscleIds;
    Box mbb;

	MatrixXd m;

};

#endif