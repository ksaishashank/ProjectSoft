#include "PlayBackFrame.h"
#include "ForceRecorder.h"

PlayBackFrame::PlayBackFrame() : mVersion(0)
{
	mForces = new ForceRecorder();
}

PlayBackFrame::PlayBackFrame(double playerVersion) : mVersion(playerVersion)
{
	mForces = new ForceRecorder();
}
PlayBackFrame::PlayBackFrame(const PlayBackFrame& rhs) : mVersion(0)
{
	mForces = new ForceRecorder();
	copyFrom(rhs);
}

const PlayBackFrame& PlayBackFrame::operator= (const PlayBackFrame& rhs)
{
	if (this == &rhs) return *this;
	copyFrom(rhs);
	return *this;
}

PlayBackFrame::~PlayBackFrame()
{
	if (mForces)
		delete mForces;
}

void PlayBackFrame::SetPlayerVersion(double version)
{
	mVersion = version;
}

void PlayBackFrame::copyFrom(const PlayBackFrame& rhs)
{
	mVersion = rhs.mVersion;
	mNodes = rhs.mNodes;
	*mForces = *(rhs.mForces);
	mMuscleFibers = rhs.mMuscleFibers;
}

DecoArchive& operator<< (DecoArchive& Ar, const PlayBackFrame& frame)
{
	Ar << frame.mNodes;
	frame.mForces->SetPlayerVersion(frame.mVersion);
	Ar << *(frame.mForces);
	if (frame.mVersion >= 0.5)
		Ar << frame.mMuscleFibers;
	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, PlayBackFrame& frame)
{
	Ar >> frame.mNodes;
	frame.mForces->SetPlayerVersion(frame.mVersion);
	Ar >> *(frame.mForces);
	if (frame.mVersion >= 0.5)
		Ar >> frame.mMuscleFibers;
	return Ar;
}

