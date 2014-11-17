#ifndef _DECO_SOFT_BODY_H
#define _DECO_SOFT_BODY_H

#include "SceneObj.h"
#include "TetMesh.h"
#include "ODESolver.h"
#include "Eigen/Dense"
#include "utility/BlockSparseMatrix.h"
#include "FEMVertexConstraint.h"
#include "ForceRecorderRenderType.h"
#include "DecoSoftObjectNode.h"

using namespace Eigen;

class SoftBodyVertexConstraint;
class ForceRecorder;
class PlayBackFrame;

class DecoSoftObject : public DecoSceneObject
{   
	friend class ForceRecorder;
public:
	DecoSoftObject();
	DecoSoftObject(const DecoSoftObject& rhs);
	virtual DecoSoftObject& operator= (const DecoSoftObject& rhs);
	virtual ~DecoSoftObject();
	virtual void SetYoungsModulus(double modulus);
	virtual void SetPoissonRatio(double poissonRatio);
	virtual double GetYoungsModulus() const;
    virtual void SetDampingCoeff(double damp1, double damp2);
	virtual void SetDensity(double density);
	virtual void SetFrictionCoeff(double friction, bool isAnisotropic = false);
	virtual bool IsFrictionAnisotropic() const;
	virtual BOOL CreateSceneObjectFromMesh(const std::string & path, double timeStep = 0.03333);
	virtual DecoRenderData* GetRenderData();
	virtual BOOL Move(const vector3& offset);
    virtual BOOL MoveDeformedShape(const vector3& offset);
	virtual BOOL Rotate(AXIS axis, float rad, BOOL bAlongGlobal = TRUE);
	virtual BOOL RotateAlongGlobalVector(vector3 axis, FLOAT rad);
    virtual BOOL RotateDeformedShape(const vector3& axis, double rad);
	virtual BOOL Scale(const vector3& fac);
    virtual BOOL Stretch(const vector3& fac);
	virtual BOOL SetPosition(const vector3& pos);
	virtual vector3 SampleVelocity(const vector3& pos);
	virtual void Update(DOUBLE deltaTime);
    virtual int GetNumElements() const;
	virtual int GetDim() const;
	virtual int GetNumDofs() const;
	virtual int GetNumUnConstrainedDofs() const;
	virtual int GetNumNodes() const;
	virtual DecoSoftObjectNode GetNode(int ithNode) const;
	virtual void SetInitialVelocity(const vector3& v);
	virtual void SetState(double* state);
	virtual void GetState(double* state) const;
	virtual void CalculateDeriv(double timeStep, double* deriv = NULL);
	virtual Eigen::VectorXd CalculateElasticForce(double timeStep);	
	virtual Eigen::VectorXd CalculateGravityForce(double timeStep);
	virtual Eigen::VectorXd CalculateElasticStress(double timeStep);

	virtual void ExertForceOnNode(int nodeIndex, const vector3& force);
	virtual void ExertForceOnNode(const VectorXd& force);
	void ExertPerturbationForceOnNode(int nodeIndex, const vector3& force);
	void ExertFrictionForceOnNode(int nodeIndex, const vector3& force);
	void ExertNormalContactForceOnNode(int nodeIndex, const vector3& force);
	virtual double GetFrictionCoeff() const
	{
		return mFrictionCoeff;
	}
	virtual void AddConstraint(SoftBodyVertexConstraint* constraint);
	virtual void RemoveAllConstraints();
	virtual void RemoveConstrainedDof(BlockSparseMatrix& mat) const;
	virtual void RemoveConstrainedDof(VectorXd& vec) const;
	virtual void RemoveRowConstrainedDof(MatrixXd& mat) const;
	virtual void RemoveColConstrainedDof(MatrixXd& mat) const;
	virtual void SetZeroConstrainedDof(VectorXd& vec) const;
	virtual void SetZeroColumnConstrainedDof(MatrixXd& mat) const;
	virtual void AddConstrainedDof(VectorXd& vec) const;
	virtual void ClearAllConstraints();
	virtual const BlockSparseMatrix& GetMassMatrix();
	virtual const BlockSparseMatrix& GetAugmentedMassMatrix(double timeStep);
    virtual const BlockSparseMatrix& GetDampingMatrix();
	virtual const BlockSparseMatrix& GetDfDx(); //only for elastic force
    virtual const BlockSparseMatrix& GetDfDxFiniteDiff(); //used for debugging
	virtual BlockSparseMatrix& GetReferenceMassMatrix(); //for implicit solver

	virtual Vector3d GetLinearMomentum();
	virtual Vector3d GetAngularMomentum();
    virtual Vector3d GetTorque();
    virtual double GetNodalMass(int ithNode) const;
	virtual void GetForce(Eigen::VectorXd& force) const;
	virtual void GetVel(Eigen::VectorXd& vel) const;
	virtual void GetPos(Eigen::VectorXd& pos) const;
	virtual void GetPosLast(Eigen::VectorXd& pos) const;
	virtual void GetPosRest(Eigen::VectorXd& pos) const;
	virtual void SetVel(const Eigen::VectorXd& vel);
	virtual void SetPos(const Eigen::VectorXd& pos);
    virtual void SetIthPos(int ithVertex, const Vector3d& pos);
    virtual double GetGravityConstant() const;
    virtual void SetGravityConstant(double gravity);
	virtual double GetMass() const;
    virtual vector3 GetIthVertex(int i) const;
    virtual vector3 GetIthVertexRest(int i) const;
	virtual vector3 GetIthVertexVel(int i) const;
	virtual TetMesh* GetRestShapeTet() 
	{
		return mRestShape;
	}
	virtual TetMesh* GetDeformedShapeTet()
	{
		return mDeformedShape;
	}
	virtual Vector3d GetCenterOfPressure() const;
	virtual Vector3d GetCenterOfPressureLast() const;
    virtual Vector3d GetCenterOfMass() const;
	virtual Vector3d GetVelOfCOM() const;
	virtual Vector3d GetVelOfCOP() const;
	virtual Vector3d GetCenterOfMassRest() const;
	virtual Vector3d GetCenterOfMassLast() const;
    virtual const MatrixXd& GetCenterOfMassMultiplier() const; //com = multiplier^T * p
	virtual void ConstructTestConstraint();
	void SetRenderOption(SoftObjRenderOption option);
	bool SelectPoints(const vector3& origin, const vector3& dir, double& depth, int& vertexId);
	void SetSelectVertex(int id, bool bAppend);
    MatrixXd CalculateDfDXPerTet(int ithTet);
    MatrixXd CalculateDfDxPerTetFiniteDiff(int ithTet);
    MatrixXd CalculateElasticForcePerTet(int ithTet);
	void UpdateCenterOfPressure(const set<int>& contactTriangleIndices, const set<int>& contactNodeIndices);
#ifdef _FRICTION_CONTACT
	double GetAnisotropicFrictionScale(const vector3& dir);
#endif
	void PushForceAccumulator();
	void PopForceAccumulator();
	virtual void DrawForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void ClearContactForce();
	void ClearPerturbationForce();
	void ToggleForceRenderOption(ForceRecorderRenderType type);
    virtual void RecordMovie(DecoArchive& Ar, double playerVersion) const;
	virtual void SetInitialPoseForMovie(DecoArchive& Ar, double playerVersion);
	virtual void CacheMovie(DecoArchive& Ar, double playerVersion);
	virtual void PlaybackMovie(int ithFrame);
	virtual void Serialize(DecoArchive& Ar) const;
	virtual void Deserialize(DecoArchive& Ar);
    
	ForceRecorder* mForces; //for rendering and debug only
protected:
	virtual void copyFrom(const DecoSoftObject& rhs);
	virtual void accumulateForce(double timeStep);
	virtual void preUpdate(double timeStep);
	virtual void postUpdate(double timeStep);
	virtual matrix33 stressStrainRelation(const matrix33& strain);
	virtual matrix33 dampingStrainDotRelation(const matrix33& strainDot);
	virtual matrix33 calculateStressAndDampingForIthTet(int i);
	virtual void computeMass();
	virtual void precomputation();
	virtual void clearForceAccumulator();
	Vector3d fromStressToForce(const Matrix3d& stress, const Tetrahedron& tet, int ithFace);
	Vector3d fromStressToForce(const matrix33& stress, const Tetrahedron& tet, int ithFace);
	
    void distributeDfDxToVertices(int ithTet, const MatrixXd& dfdxPerTet);
    vector3 calculateAreaNormal(const Tetrahedron& tet, int ithFace);
    matrix33 calculateDStrainDXComponent(int ithTet, int ithVertex, int ithComponent);
    vector3 calculateDAreaNormalDxComponent(int ithTet, int ithFace, int ithVertex, int ithComponent);

	TetMesh* mRestShape;
	TetMesh* mDeformedShape;

	std::vector<DecoSoftObjectNode> mNodes;
	std::vector<matrix33> mRestShapeInvertedX;
	double mFrictionCoeff;
    
	double mRho;
	double mPoissonRatio;
	double mYoungsModulus;
	double mDampingConstant1;//analogues to Youngs modulus // for corotational linear FEM, this is for coeff of mass
	double mDampingConstant2;//analogues to Poisson ratio  // for corotational linear FEM, this is for coeff of stiffness
	bool mbNeedRecalculateForce;
	double mMass;
	BlockSparseMatrix mMassMatrix;
    BlockSparseMatrix mDampingMatrix;
	BlockSparseMatrix mAugmentedMassMatrix;
    BlockSparseMatrix mDfDx;
	ODESolver* mSolver;
	
	std::vector<SoftBodyVertexConstraint*> mConstraints;
	SoftObjRenderOption mRenderOption;
	bool mIsFrictionAnisotropic;
    MatrixXd mCenterOfMassMulitplier;
	vector<PlayBackFrame> mPlaybackMovie;
	Vector3d mCOP;
	Vector3d mCOPLast;
	double mTimeStep;
    double mGravityConstant;

};

#endif