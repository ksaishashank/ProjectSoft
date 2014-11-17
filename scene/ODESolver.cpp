#include "stdafx.h"
#include "ODESolver.h"
#include "DecoSoftBody.h"
#ifdef _MKL_IMPLEMENT
#include <mkl_cblas.h>
#include <mkl_spblas.h>
#include <mkl_rci.h>
#include <mkl_blas.h>
#include <mkl_service.h>
#else 
#include "utility/JieBlas.h"
#endif
#include "utility/DecoLogger.h" 
#include "utility/BlockSparseMatrix.h"
#include "Eigen/Dense"

void ODESolver::Solve(double timeStep)
{
//	mkl_domain_set_num_threads( 1, MKL_ALL );

	switch(mType)
	{
	case ST_ExplicitEuler:
		ExplicitEulerStep(timeStep);
		break;
	case ST_RK4:
		RK4Step(timeStep);
		break;
	case ST_ImplicitEuler:
		ImplicitEulerStep(timeStep);
		break;
	default:
		assert(0);
	}

}
void ODESolver::ExplicitEulerStep(double timeStep)
{
	size_t dim = mDynamicSys->GetDim();
	double* deriv = new double[dim];
	mDynamicSys->CalculateDeriv(timeStep, deriv);
	double* currentState = new double[dim];
	mDynamicSys->GetState(currentState);

#ifdef _MKL_IMPLEMENT
	cblas_daxpy(static_cast<int>(dim), timeStep, deriv, 1, currentState, 1);
#else
	myblas_daxpy(static_cast<int>(dim), timeStep, deriv, currentState);
#endif
	mDynamicSys->SetState(currentState);
	delete[] deriv;
	delete[] currentState;
}


void ODESolver::RK4Step(double timeStep)
{
	int dim = static_cast<int>(mDynamicSys->GetDim());
	double* currentState = new double[dim];
	double* tmpState = new double[dim];
	double* k1 = new double[dim];
	double* k2 = new double[dim];
	double* k3 = new double[dim];
	double* k4 = new double[dim];


	mDynamicSys->GetState(currentState);
	mDynamicSys->CalculateDeriv(timeStep, k1);
#ifdef _MKL_IMPLEMENT
	cblas_dcopy(dim, currentState, 1, tmpState, 1);
	cblas_daxpy(dim, timeStep / 2, k1, 1, tmpState, 1);
#else
	myblas_dcopy(dim, currentState, tmpState);
	myblas_daxpy(dim, timeStep / 2, k1, tmpState);

#endif
	mDynamicSys->SetState(tmpState);

	mDynamicSys->CalculateDeriv(timeStep, k2);
#ifdef _MKL_IMPLEMENT
	cblas_dcopy(dim, currentState, 1, tmpState, 1);
	cblas_daxpy(dim, timeStep / 2, k2, 1, tmpState, 1);
#else
	myblas_dcopy(dim, currentState, tmpState);
	myblas_daxpy(dim, timeStep / 2, k2, tmpState);
#endif
	mDynamicSys->SetState(tmpState);

	mDynamicSys->CalculateDeriv(timeStep, k3);
#ifdef _MKL_IMPLEMENT
	cblas_dcopy(dim, currentState, 1, tmpState, 1);
	cblas_daxpy(dim, timeStep, k3, 1, tmpState, 1);
#else
	myblas_dcopy(dim, currentState, tmpState);
	myblas_daxpy(dim, timeStep, k3, tmpState);
#endif
	mDynamicSys->SetState(tmpState);
 
	mDynamicSys->CalculateDeriv(timeStep, k4);

#ifdef _MKL_IMPLEMENT
	cblas_daxpy(dim, 1.f / 6.f * timeStep, k1, 1, currentState, 1);
	cblas_daxpy(dim, 1.f / 3.f * timeStep, k2, 1, currentState, 1);
	cblas_daxpy(dim, 1.f / 3.f * timeStep, k3, 1, currentState, 1);
	cblas_daxpy(dim, 1.f / 6.f * timeStep, k4, 1, currentState, 1);
#else
	myblas_daxpy(dim, 1.f / 6.f * timeStep, k1, currentState);
	myblas_daxpy(dim, 1.f / 3.f * timeStep, k2, currentState);
	myblas_daxpy(dim, 1.f / 3.f * timeStep, k3, currentState);
	myblas_daxpy(dim, 1.f / 6.f * timeStep, k4, currentState);
#endif
	mDynamicSys->SetState(currentState);


	delete[] k1;
	delete[] k2;
	delete[] k3;
	delete[] k4;
	delete[] currentState;
	delete[] tmpState;
}

void ODESolver::ImplicitEulerStep(double timeStep)
{
	DecoTimer timer;
	timer.startTimer();

	int dim = static_cast<int>(mDynamicSys->GetDim());
	int forceDim = dim / 2;
	double* deriv = new double[dim];
	mDynamicSys->CalculateDeriv(timeStep, deriv); //deriv not used, just to let dfdx, force, etc. to be calculated.
	double time1 = timer.getCurrentTime();
	timer.resetTimer();

	Eigen::VectorXd force, currentVel, currentPos, nextPos;

	const BlockSparseMatrix& dfdx = mDynamicSys->GetDfDx();
	const BlockSparseMatrix& mass = mDynamicSys->GetMassMatrix();	


	mDynamicSys->GetForce(force);
	mDynamicSys->GetVel(currentVel);
	mDynamicSys->GetPos(currentPos);
    
    double time2 = timer.getCurrentTime();


	timer.resetTimer();
	Eigen::VectorXd rhs = mass * currentVel + timeStep * force;

	Eigen::VectorXd nextVel(rhs.size());
#if LLT_SOLVE
	BlockSparseMatrix& refLhs  = mDynamicSys->GetReferenceMassMatrix();
	refLhs.LltSolve(rhs, nextVel);
#else
	BlockSparseMatrix lhs = mDynamicSys->GetAugmentedMassMatrix(timeStep);
	mDynamicSys->RemoveConstrainedDof(lhs);
	mDynamicSys->RemoveConstrainedDof(rhs);

	lhs.ConjugateGradientSolve(rhs, nextVel);
#endif
	double time3 = timer.getCurrentTime();
//	mDynamicSys->RemoveConstrainedDof(nextVel);
	mDynamicSys->AddConstrainedDof(nextVel);
	nextPos = currentPos + timeStep * nextVel;
	mDynamicSys->SetVel(nextVel);
	mDynamicSys->SetPos(nextPos);
	delete[] deriv;

    //LOG(INFO) << "CalculateDeriv took " << time1 << "seconds.";
    //LOG(INFO) << "Retrive Info took " << time2 << " seconds.";
    //LOG(INFO) << "Linear solver took " << time3 << " seconds.";

}