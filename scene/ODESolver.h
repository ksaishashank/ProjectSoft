#ifndef _ODE_SOLVER_H
#define _ODE_SOLVER_H

class DecoSoftObject;

enum SolverType
{
	ST_ExplicitEuler,
	ST_RK4,
	ST_ImplicitEuler,
};

class ODESolver
{
public:
	ODESolver(SolverType type, DecoSoftObject* psys) : mType(type), mDynamicSys(psys)
	{
	}
	void Solve(double timeStep);
private:
	SolverType mType;
	DecoSoftObject* mDynamicSys;

	void ExplicitEulerStep(double timeStep);
	void RK4Step(double timeStep);
	void ImplicitEulerStep(double timeStep);


};


#endif