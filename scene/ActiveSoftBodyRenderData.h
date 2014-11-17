#ifndef _ACTIVE_SOFT_BODY_RENDER_DATA
#define _ACTIVE_SOFT_BODY_RENDER_DATA

#include "MuscleFiber.h"
#include "TetMesh.h"
#include <vector>
using namespace std;

class ActiveSoftBodyRenderData : public TetMesh
{
public:
	ActiveSoftBodyRenderData();
	void SetMuscleFibers(vector<MuscleFiber>* fibers);
    void SetSelectedMuscles(const vector<int>& selectedMuscles);
	virtual ~ActiveSoftBodyRenderData();
	virtual void RenderOpaqueSection(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType);
private:
	
	vector<MuscleFiber>* mMuscleFibers;
    vector<int> mSelectedMuscleIds;
};

#endif