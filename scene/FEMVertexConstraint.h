#ifndef _FEM_VERTEX_CONSTRAINT_H
#define _FEM_VERTEX_CONSTRAINT_H

#include "DecoSoftBody.h"
#include <vector>
#include <Eigen/Dense>
#include <utility/archive.h>
using namespace std;
using namespace Eigen;


class DecoSoftObject;

class SoftBodyVertexConstraint
{
public:
	SoftBodyVertexConstraint(DecoSoftObject* obj);
	
	virtual ~SoftBodyVertexConstraint();
	virtual void SetObject(DecoSoftObject* obj);
	virtual void AddVertex(int idx);
    virtual int GetNumConstrainedVertices() const;
	virtual void RemoveConstrainedDof(BlockSparseMatrix& mat) const;
	virtual void RemoveConstrainedDof(VectorXd& vec) const;
	virtual void RemoveRowConstrainedDof(MatrixXd& mat) const;
	virtual void RemoveColConstrainedDof(MatrixXd& mat) const;
	virtual void SetZeroConstrainedDof(VectorXd& vec) const;
	virtual void SetZeroColumnConstrainedDof(MatrixXd& mat) const;
	virtual void AddConstrainedDof(VectorXd& vec) const;
	virtual void Serialize(DecoArchive& Ar) const;
	virtual void Deserialize(DecoArchive& Ar);
protected:
	DecoSoftObject* mObject;
	vector<int> mVertexIndices;
	vector<int> mConstrainedTable;

};

#endif