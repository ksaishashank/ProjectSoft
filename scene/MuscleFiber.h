#ifndef _MUSCLE_FIBER_H
#define _MUSCLE_FIBER_H

#include <vector>
#include <Eigen/Dense>
#include "utility/archive.h"
using namespace std;
using namespace Eigen;

class ActiveSoftBody;
class MuscleDof;

class MuscleFiber
{
public:
	MuscleFiber();
	MuscleFiber(const vector<Vector3d>& vertices);
	virtual ~MuscleFiber();

	virtual int GetNumSegments() const;
	virtual const vector<Vector3d>& GetPositions() const;
	virtual void SetSoftBody(ActiveSoftBody* softBody);
	virtual void SetPosition(const vector<Vector3d>& pos);
	virtual void SetRestShapePosition(const vector<Vector3d>& pos);
	virtual void SetTetIdx(const vector<int>& indices);
	virtual void SetBarycentricCoord(const vector<Vector3d>& coord);
	virtual Matrix3d GetRotationFromXAxis(int segmentId);
	virtual Matrix3d Contract(int segmentId, double magnitude);
	virtual void UpdatePosition();	
	virtual void UpdateBarycentricCoordAndTetIndex();
	virtual const vector<Vector3d>& GetPolylineRepresentation() const;
	const vector<Vector3d>& GetPolylineRepresentationRestShape() const;
	virtual double GetCurrentLength(int ithSegment) const;
	virtual double GetRestLength(int ithSegment) const;

    void SetCurrentAsRest();
	void Translate(const Vector3d& offset);
	double GetInfluenceSigma() const;
	void SetInfluenceSigma(double sigma);
	void ConstructTestLongitudinalFiber(ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, int ithFiber);
	void ConstructTestRadialHorizontalFiber(ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, int ithFiber);
	void ConstructTestRadialVerticalFiber(ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, int ithFiber);
	void ConstructTestTwistFiber(ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, int ithFiber);
	void AddDof(MuscleDof* dof);
    int ReadFromFile(ifstream& in, ActiveSoftBody* softBody, vector<MuscleDof>& muscleDofs, vector<MuscleDof>& specialDofs, int ithFiber);
	friend DecoArchive& operator<< (DecoArchive& Ar, const MuscleFiber& fiber);
	friend DecoArchive& operator>> (DecoArchive& Ar, MuscleFiber& fiber);
private:
	vector<Vector3d> mVertices;
	vector<Vector3d> mVerticesRestShape;
	vector<Vector3d> mBaryCentricCoordInTet;
	vector<int> mTetIndex;
	vector<MuscleDof*> mDofs;
	double mInfluenceSigma;
	ActiveSoftBody* mSoftBody;
};

#endif