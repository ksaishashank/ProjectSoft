#include "ActiveSoftBodyRenderData.h"
#include "render/RenderMiscs.h"

ActiveSoftBodyRenderData::ActiveSoftBodyRenderData() : mMuscleFibers(NULL)
{

}
void ActiveSoftBodyRenderData::SetMuscleFibers(vector<MuscleFiber>* fibers)
{
	mMuscleFibers = fibers;
}
ActiveSoftBodyRenderData::~ActiveSoftBodyRenderData()
{

}

void ActiveSoftBodyRenderData::SetSelectedMuscles(const vector<int>& selectedMuscles)
{
    mSelectedMuscleIds = selectedMuscles;
}

void ActiveSoftBodyRenderData::RenderOpaqueSection(DecoRenderInterface* RI, DecoLight** Lights, INT numLight, DecoDrawType DrawType)
{
	if ((mRenderOption == RO_Points|| mRenderOption == RO_Edges) && mMuscleFibers && !mMuscleFibers->empty())
	{
		int numMuscleFibers = static_cast<int>(mMuscleFibers->size());
		for (int i = 0; i < numMuscleFibers; ++i)
		{
            float stroke = 2;
            if (find(mSelectedMuscleIds.begin(), mSelectedMuscleIds.end(), i) != mSelectedMuscleIds.end())
                stroke = 4;
			vector<vector3> points;
			vector<DecoColor> colors;
			const vector<Vector3d> vertices = (*mMuscleFibers)[i].GetPositions();
			int numVertices = static_cast<int>(vertices.size());
			for (int ithVert = 0; ithVert < numVertices; ++ithVert)
			{
				points.push_back(vector3(vertices[ithVert](0), vertices[ithVert](1), vertices[ithVert](2)));
				colors.push_back(0xff0000ff);
			}
			DecoRenderMisc::GetSingleton()->DrawLineStrip(&(points[0]), &(colors[0]), points.size(), stroke);
		}
	}
	TetMesh::RenderOpaqueSection(RI, Lights, numLight, DrawType);
}