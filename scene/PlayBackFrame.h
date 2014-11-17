#ifndef PlayBackFrame_H
#define PlayBackFrame_H

#include "DecoSoftObjectNode.h"
#include "ActiveSoftBody.h"

class ForceRecorder;

class PlayBackFrame
{
public:
	PlayBackFrame();
	PlayBackFrame(double playerVersion);
	PlayBackFrame(const PlayBackFrame& rhs);
	~PlayBackFrame();
	const PlayBackFrame& operator= (const PlayBackFrame& rhs);
	void SetPlayerVersion(double version);
	vector<DecoSoftObjectNode> mNodes;
	vector<MuscleFiber> mMuscleFibers;
	ForceRecorder* mForces;
	friend DecoArchive& operator<< (DecoArchive& Ar, const PlayBackFrame& node);
	friend DecoArchive& operator>> (DecoArchive& Ar, PlayBackFrame& node);
private:
	void copyFrom(const PlayBackFrame& rhs);
	double mVersion;
};


#endif