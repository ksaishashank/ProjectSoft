#ifndef _FEM_SIMULATOR
#define _FEM_SIMULATOR

#include "TetMesh.h"


class FEMSimulator
{
public:
	FEMSimulator();
	~FEMSimulator();

	void Simulate(double timeStep);

};


#endif