#include "grid.h"
#include <glog/logging.h>
#include "render/RenderMiscs.h"
using namespace google;

Grid::Grid()
{

}
Grid::~Grid()
{

}
void Grid::SetResolution(int xRes, int yRes, int zRes)
{
    mNumGridsXDir = xRes;
    mNumGridsYDir = yRes;
    mNumGridsZDir = zRes;
}

void Grid::GetResolution(int& xRes, int& yRes, int& zRes) const
{
    xRes = mNumGridsXDir;
    yRes = mNumGridsYDir;
    zRes = mNumGridsZDir;
}

vector3 Grid::GetCenter() const
{
    return mBoundingBox.GetCenter();
};

void Grid::Move(double x, double y, double z)
{
    mBoundingBox.Move(x, y, z);
}

void Grid::SetBoundingBox(const Box& box)
{
    mBoundingBox = box;
}
const Box& Grid::GetBoudingBox() const
{
    return mBoundingBox;
}

vector3 Grid::GetCellSize() const
{
    vector3 sceneBoxEdgeLength = mBoundingBox.GetExtent() * 2;
    double stepSizeX = sceneBoxEdgeLength.x / mNumGridsXDir;
    double stepSizeY = sceneBoxEdgeLength.y / mNumGridsYDir;
    double stepSizeZ = sceneBoxEdgeLength.z / mNumGridsZDir;
    return vector3(stepSizeX, stepSizeY, stepSizeZ);
}
 
Box Grid::GetCellBox(int i, int j, int k) const
{

    vector3 sceneBoxMinCorner = mBoundingBox.GetMin();
    vector3 stepSize = GetCellSize();
    vector3 retMinCorner = sceneBoxMinCorner + vector3(i * stepSize.x, j * stepSize.y, k * stepSize.z);
    vector3 retMaxCorner = retMinCorner + vector3(stepSize.x, stepSize.y, stepSize.z);
    return Box(retMinCorner, retMaxCorner);

}

Box Grid::GetCellBox(const Index3& index) const
{
    return GetCellBox(index.m_i, index.m_j, index.m_k);
}

Index3 Grid::InCellId(const vector3& pt) const
{
    Index3 ret;
    vector3 stepSize = GetCellSize();
    double stepX = stepSize.x;
    double stepY = stepSize.y;
    double stepZ = stepSize.z;
    vector3 offsetFromMin = pt - mBoundingBox.GetMin();
    ret.m_i = static_cast<int>(offsetFromMin.x / stepX);
    ret.m_j = static_cast<int>(offsetFromMin.y / stepY);
    ret.m_k = static_cast<int>(offsetFromMin.z / stepZ);
    return ret;
}

vector<Index3> Grid::GetCellBoxIdIntersectAlignedLine(const vector3& start, const vector3& end, AXIS axis) const
{
    vector<Index3> ret;
    Index3 startId = InCellId(start);
    Index3 endId = InCellId(end);
    
    switch(axis)
    {
    case AXIS_X:
        for (int i = endId.m_i; i <= startId.m_i; ++i)
        {
            ret.push_back(Index3(i, startId.m_j, startId.m_k));
        }
        break;
    case AXIS_Y:
        for (int j = endId.m_j; j <= startId.m_j; ++j)
        {
            ret.push_back(Index3(startId.m_i, j, startId.m_k));
        }
        break;
    case AXIS_Z:
        for (int k = endId.m_k; k <= startId.m_k; ++k)
        {
            ret.push_back(Index3(startId.m_i, startId.m_j, k));
        }
        break;
    default:
        LOG(ERROR) << "The axis does not exist.";
    }

    return ret;
}

vector<Box> Grid::GetCellBoxIntersectAlignedLine(const vector3& start, const vector3& end, AXIS axis) const
{
    vector<Index3> boxId = GetCellBoxIdIntersectAlignedLine(start, end, axis);
    vector<Box> boxes;
    int numBoxes = static_cast<int>(boxId.size());
    for (int i = 0; i < numBoxes; ++i)
    {
        boxes.push_back(GetCellBox(boxId[i]));
    }
    return boxes;
}

void Grid::GetWallFractionOfLine(int i, int j, int k, const vector3& start, const vector3& end, AXIS axis, double& alpha, double &beta) const 
{
    Box cellBox = GetCellBox(Index3(i, j, k));
    double startValue, endValue;
    double cellWall1, cellWall2;

    vector3 cellSizeVector = GetCellSize();
    switch(axis)
    {
    case AXIS_X:
        startValue = start.x;
        endValue = end.x;
        cellWall1 = cellBox.GetMin().x;
        cellWall2 = cellBox.GetMax().x;
        break;
    case AXIS_Y:
        startValue = start.y;
        endValue = end.y;
        cellWall1 = cellBox.GetMin().y;
        cellWall2 = cellBox.GetMax().y;
        break;
    case AXIS_Z:
        startValue = start.z;
        endValue = end.z;
        cellWall1 = cellBox.GetMin().z;
        cellWall2 = cellBox.GetMax().z;
        break;
    default:
        LOG(ERROR) << "The axis does not exist.";
    }
    //I think start > end
    alpha = (cellWall2 - endValue) / (startValue - endValue);
    beta = (cellWall1 - endValue) / (startValue - endValue);
    alpha = Clamp(alpha, 0.0, 1.0);
    beta = Clamp(beta, 0.0, 1.0);
}

double Grid::GetFractionInCell(int i, int j, int k, const vector3& start, const vector3& end, AXIS axis) const
{
    double ret;
    double length;
    double startValue, endValue;
    double cellSize;
    double cellWall1, cellWall2;
    Box cellBox = GetCellBox(Index3(i, j, k));
    vector3 cellSizeVector = GetCellSize();
    switch(axis)
    {
    case AXIS_X:
        startValue = min(start.x, end.x);
        endValue = max(start.x, end.x);
        cellWall1 = cellBox.GetMin().x;
        cellWall2 = cellBox.GetMax().x;
        cellSize = cellSizeVector.x;
        break;
    case AXIS_Y:
        startValue = min(start.y, end.y);
        endValue = max(start.y, end.y);
        cellWall1 = cellBox.GetMin().y;
        cellWall2 = cellBox.GetMax().y;
        cellSize = cellSizeVector.y;
        break;
    case AXIS_Z:
        startValue = min(start.z, end.z);
        endValue = max(start.z, end.z);
        cellWall1 = cellBox.GetMin().z;
        cellWall2 = cellBox.GetMax().z;
        cellSize = cellSizeVector.z;
        break;
    default:
        LOG(ERROR) << "The axis does not exist.";
    }
    length = endValue - startValue;
    if (startValue < cellWall1)
        startValue = cellWall1;
    if (endValue > cellWall2)
        endValue = cellWall2;
    double lengthInCell = endValue - startValue;
    ret = lengthInCell / (length + EPSILON_FLOAT);
    return ret;

}
    

void Grid::Render()
{
    for (int i = 0; i < mNumGridsXDir; ++i)
    {
        for (int j = 0; j < mNumGridsYDir; ++j)
        {
            for (int k = 0; k < mNumGridsZDir; ++k)
            {
                Box b = GetCellBox(i, j, k);
                DecoRenderMisc::GetSingleton()->DrawBox(NULL, b);
            }
        }
    }
}