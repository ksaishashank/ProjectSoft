#ifndef _GRID_H
#define _GRID_H

#include "utility/mathlib.h"

class Grid
{
public:
    Grid();
    ~Grid();
    void Move(double x, double y, double z);
    void SetResolution(int xRes, int yRes, int zRes);
    void GetResolution(int& xRes, int& yRes, int& zRes) const;
    void SetBoundingBox(const Box& box);
    const Box& GetBoudingBox() const;
    vector3 GetCenter() const;
    vector3 GetCellSize() const;
    Index3 InCellId(const vector3& pt) const;
    Box GetCellBox(int i, int j, int k) const;
    Box GetCellBox(const Index3& index) const;
    vector<Index3> GetCellBoxIdIntersectAlignedLine(const vector3& start, const vector3& end, AXIS axis) const;
    vector<Box> GetCellBoxIntersectAlignedLine(const vector3& start, const vector3& end, AXIS axis) const;
    double GetFractionInCell(int i, int j, int k, const vector3& start, const vector3& end, AXIS axis) const;
    void GetWallFractionOfLine(int i, int j, int k, const vector3& start, const vector3& end, AXIS axis, double& alpha, double &beta) const;
    void Render();
private:
    int mNumGridsXDir;
    int mNumGridsYDir;
    int mNumGridsZDir;
    Box mBoundingBox;

};

#endif