#ifndef _MUSCLE_DOF_H
#define _MUSCLE_DOF_H

#include "ActiveSoftBody.h"

class MuscleDof
{
public:
	MuscleDof();
	MuscleDof(ActiveSoftBody* obj);
	~MuscleDof();
	void SetSoftBody(ActiveSoftBody* obj);
	void BindWithMuscleFiber(int ithFiber, int startSegId, int numSegs);
	double GetCurrentLength() const;
	double GetRestLength() const;
	double GetCurrentLengthRelativeToRest() const;
	const vector<int>& GetFiberIndex() const;
	vector<MuscleFiber*> GetFiber() const;
	ActiveSoftBody* GetSoftBody() const;
	vector<int> mIthFiber;
	vector<int> mStartSegmentIdx;
	vector<int> mNumSegmentInControl;
	double mValue;
	friend DecoArchive& operator<< (DecoArchive& Ar, const MuscleDof& dof);
	friend DecoArchive& operator>> (DecoArchive& Ar, MuscleDof& dof);
private:
	ActiveSoftBody* mSoftBody;
};

#endif