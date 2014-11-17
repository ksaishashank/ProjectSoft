#include "DecoSoftObjectNode.h"
#include "utility/archive.h"

void DecoSoftObjectNode::Serialize(DecoArchive& Ar) const
{
	Ar << mPos;
	Ar << mVel;
	Ar << mForceAccumulator;
	Ar << mMass;

}
void DecoSoftObjectNode::Deserialize(DecoArchive& Ar)
{
	Ar >> mPos;
	Ar >> mVel;
	Ar >> mForceAccumulator;
	Ar >> mMass;
}

DecoArchive& operator<< (DecoArchive& Ar, const DecoSoftObjectNode& node)
{
	node.Serialize(Ar);
	return Ar;
}

DecoArchive& operator>> (DecoArchive& Ar, DecoSoftObjectNode& node)
{
	node.Deserialize(Ar);
	return Ar;
}