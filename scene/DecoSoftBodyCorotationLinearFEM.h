#ifndef _DECO_SOFT_BODY_COROTATION_LINEAR_FEM
#define _DECO_SOFT_BODY_COROTATION_LINEAR_FEM

#include "DecoSoftBody.h"
#include "Eigen/Dense"
#include "utility/BlockSparseMatrix.h"



class DecoSoftObjectCorotationLinearFEM : public DecoSoftObject
{
public:
	DecoSoftObjectCorotationLinearFEM();
	virtual ~DecoSoftObjectCorotationLinearFEM();
	virtual Eigen::VectorXd CalculateElasticForce(double timeStep);	
	virtual const BlockSparseMatrix& GetDfDx();
	virtual const BlockSparseMatrix& GetDampingMatrix();
	virtual const BlockSparseMatrix& GetAugmentedMassMatrix(double timeStep);
	virtual const BlockSparseMatrix& GetMk();
	virtual const BlockSparseMatrix& GetMf();
	virtual const VectorXd& GetMx() const;
	virtual Eigen::VectorXd CalculateElasticStress(double timeStep);

#if LLT_SOLVE
	virtual BlockSparseMatrix& GetReferenceMassMatrix();
#endif

protected:
	virtual void accumulateForce(double timeStep);
	virtual void precomputation();
    bool canbeLazy();    
	void assembleK(double timeStep);
	void formatPositionToEigenVector(const TetMesh& mesh, Eigen::VectorXd& vector);
	void planInternalForce();
	void calculateBPerTet();
	void calculateKPerTet();
    MatrixXd calculateDStrainDx(int ithTet);

	BlockSparseMatrix mK;
	BlockSparseMatrix mF;
	BlockSparseMatrix mReferenceMassForImplicitEuler;

	std::vector<Eigen::MatrixXd> mKPerTet;
    std::vector<Eigen::MatrixXd> mBPerTet;
	std::vector<double> mYoungsRatioPerTet;
	Eigen::VectorXd mX;
	Eigen::VectorXd mP;
	Eigen::MatrixXd mD;
    vector<matrix33> mLastDeformationGradient;
	vector<int> mNotLazyTetIdx;

#if LLT_SOLVE
	void updateReferenceMassMatrix(double timeStep);
	std::set<int> mRefMatrixChangedColumns;
#endif

};

#endif