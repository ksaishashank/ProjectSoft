#ifndef DECOSOFTOBJNODE_H
#define DECOSOFTOBJNODE_H

#include "utility/mathlib.h"

class DecoSoftObjectNode
{
public:
	vector3 mPos;
	vector3 mVel;
	vector3 mForceAccumulator;
	double mMass;
	vector3 mLastPos; //for debug use, not need to serialize
	vector3 mBackupForceAccumulator;

	void Serialize(DecoArchive& Ar) const;
	void Deserialize(DecoArchive& Ar);
	friend DecoArchive& operator<< (DecoArchive& Ar, const DecoSoftObjectNode& node);
	friend DecoArchive& operator>> (DecoArchive& Ar, DecoSoftObjectNode& node);
};

#endif