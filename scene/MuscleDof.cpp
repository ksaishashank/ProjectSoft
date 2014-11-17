#include "MuscleDof.h"
#include "sceneobj.h"
#include <glog/logging.h>
#include "MuscleFiber.h"
using namespace google;

MuscleDof::MuscleDof() : mSoftBody(NULL)
{

}
MuscleDof::MuscleDof(ActiveSoftBody* obj) : mSoftBody(obj)
{

}
MuscleDof::~MuscleDof()
{

}
void MuscleDof::SetSoftBody(ActiveSoftBody* obj)
{
	mSoftBody = obj;
}

void MuscleDof::BindWithMuscleFiber(int ithFiber, int startSegId, int numSegs)
{
	CHECK(mSoftBody);
	mIthFiber.push_back(ithFiber);
	mStartSegmentIdx.push_back(startSegId);
	mNumSegmentInControl.push_back(numSegs);
}

double MuscleDof::GetCurrentLengthRelativeToRest() const
{
	double currentLength = GetCurrentLength();
	double restLength = GetRestLength();
	return currentLength / restLength;
}

double MuscleDof::GetCurrentLength() const
{
    CHECK(mSoftBody);
    double currentLength = 0;
    int numFibers = static_cast<int>(mIthFiber.size());
    for (int ithFiber = 0; ithFiber < numFibers; ++ithFiber)
    {
        int fiberId = mIthFiber[ithFiber];
        const MuscleFiber& fiber = mSoftBody->GetMuscleFiber(fiberId);

        for (int i = 0; i < mNumSegmentInControl[ithFiber]; ++i)
        {
            currentLength += fiber.GetCurrentLength(mStartSegmentIdx[ithFiber] + i);
        }
    }
    return currentLength;
}
double MuscleDof::GetRestLength() const
{
	CHECK(mSoftBody);
	double currentLength = 0;
    int numFibers = static_cast<int>(mIthFiber.size());
    for (int ithFiber = 0; ithFiber < numFibers; ++ithFiber)
    {
        int fiberId = mIthFiber[ithFiber];
        const MuscleFiber& fiber = mSoftBody->GetMuscleFiber(fiberId);

	    for (int i = 0; i < mNumSegmentInControl[ithFiber]; ++i)
	    {
		    currentLength += fiber.GetRestLength(mStartSegmentIdx[ithFiber] + i);
	    }
    }
	return currentLength;
}

const vector<int>& MuscleDof::GetFiberIndex() const
{
	return mIthFiber;
}

vector<MuscleFiber*> MuscleDof::GetFiber() const
{
	CHECK(mSoftBody);
    vector<MuscleFiber*> fibers;
    int numFibers = static_cast<int>(mIthFiber.size());
    for (int ithFiber = 0; ithFiber < numFibers; ++ithFiber)
    {
	    fibers.push_back(&(mSoftBody->GetMuscleFiber(mIthFiber[ithFiber])));
    }
    return fibers;
}
ActiveSoftBody* MuscleDof::GetSoftBody() const
{
	return mSoftBody;
}

DecoArchive& operator<< (DecoArchive& Ar, const MuscleDof& dof)
{
	Ar << dof.mIthFiber;
	Ar << dof.mStartSegmentIdx;
	Ar << dof.mNumSegmentInControl;
	Ar << dof.mValue;
	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, MuscleDof& dof)
{
	Ar >> dof.mIthFiber;
	Ar >> dof.mStartSegmentIdx;
	Ar >> dof.mNumSegmentInControl;
	Ar >> dof.mValue;
	return Ar;
}
