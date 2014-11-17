#include "FEMVertexConstraint.h"
#include "DecoSoftBody.h"


SoftBodyVertexConstraint::SoftBodyVertexConstraint(DecoSoftObject* obj) : mObject(NULL)
{
	SetObject(obj);
}

SoftBodyVertexConstraint::~SoftBodyVertexConstraint()
{

}

void SoftBodyVertexConstraint::SetObject(DecoSoftObject* obj)
{
	mObject = obj;
	int numNodes = obj->GetNumNodes();
	if (mConstrainedTable.empty())
	{
		mConstrainedTable.resize(numNodes);
		for (int i = 0; i < numNodes; ++i)
		{
			mConstrainedTable[i] = 0;
		}
	}

}

int SoftBodyVertexConstraint::GetNumConstrainedVertices() const
{
    return static_cast<int>(mVertexIndices.size());
}

void SoftBodyVertexConstraint::AddVertex(int idx)
{
	mVertexIndices.push_back(idx);
	mConstrainedTable[idx] = 1;
}

void SoftBodyVertexConstraint::RemoveConstrainedDof(BlockSparseMatrix& mat) const
{
	CHECK(mObject);
	int numDofs = mObject->GetNumDofs();
	CHECK(numDofs == mat.GetNumRows() && numDofs == mat.GetNumCols());

	int numNodes = mObject->GetNumNodes();
	int numConstrainedVertices = static_cast<int>(mVertexIndices.size());
	int newNumNodes = numNodes - numConstrainedVertices;
	BlockSparseMatrix newMat;
	newMat.SetBlockInfo(3, 3, newNumNodes, newNumNodes);
	int newi = 0;
	for (int i = 0; i < numNodes; ++i)
	{
		if (mConstrainedTable[i])
			continue;
		int newj = 0;
		for (int j = 0; j < numNodes; ++j)
		{
			if (mConstrainedTable[j])
				continue;
			if (mat.IsNonZeroBlock(i, j))
				newMat.SetBlockMatrix(newi, newj, mat.GetBlockMatrix(i, j));
			newj++;
		}
		newi++;
	}

	mat = newMat;	
	mat.UpdateCSR();

}

void SoftBodyVertexConstraint::RemoveColConstrainedDof(MatrixXd& mat) const
{
	CHECK(mObject);
	int numDofs = mObject->GetNumDofs();
	CHECK(numDofs == mat.cols());

	int numNodes = mObject->GetNumNodes();
	int newNumDofs = numDofs - 3 * static_cast<int>(mVertexIndices.size());
	MatrixXd newMat = MatrixXd::Zero(mat.rows(), newNumDofs);
	int j = 0;
	for (int i = 0; i < numNodes; ++i)
	{
		if (mConstrainedTable[i]) continue;
		newMat.col(j * 3 + 0) = mat.col(i * 3 + 0);
		newMat.col(j * 3 + 1) = mat.col(i * 3 + 1);
		newMat.col(j * 3 + 2) = mat.col(i * 3 + 2);
		j++;
	}
	mat = newMat;
}

void SoftBodyVertexConstraint::RemoveRowConstrainedDof(MatrixXd& mat) const
{
	CHECK(mObject);
	int numDofs = mObject->GetNumDofs();
	CHECK(numDofs == mat.rows());

	int numNodes = mObject->GetNumNodes();
	int newNumDofs = numDofs - 3 * static_cast<int>(mVertexIndices.size());
	MatrixXd newMat = MatrixXd::Zero(newNumDofs, mat.cols());
	int j = 0;
	for (int i = 0; i < numNodes; ++i)
	{
		if (mConstrainedTable[i]) continue;
		newMat.row(j * 3 + 0) = mat.row(i * 3 + 0);
		newMat.row(j * 3 + 1) = mat.row(i * 3 + 1);
		newMat.row(j * 3 + 2) = mat.row(i * 3 + 2);
		j++;
	}
	mat = newMat;
}

void SoftBodyVertexConstraint::RemoveConstrainedDof(VectorXd& vec) const
{
	CHECK(mObject);
	int numDofs = mObject->GetNumDofs();
	CHECK(numDofs == vec.size());

	int numNodes = mObject->GetNumNodes();
	int newNumDofs = numDofs - 3 * static_cast<int>(mVertexIndices.size());
	VectorXd newVec(newNumDofs);
	int j = 0;
	for (int i = 0; i < numNodes; ++i)
	{
		if (mConstrainedTable[i]) continue;
		newVec[j * 3 + 0] = vec[i * 3 + 0];
		newVec[j * 3 + 1] = vec[i * 3 + 1];
		newVec[j * 3 + 2] = vec[i * 3 + 2];
		j++;
	}
	vec = newVec;
}
void SoftBodyVertexConstraint::SetZeroConstrainedDof(VectorXd& vec) const
{
	CHECK(mObject);
	int numDofs = mObject->GetNumDofs();
	CHECK(numDofs == vec.size());

	int numNodes = mObject->GetNumNodes();

	for (int i = 0; i < numNodes; ++i)
	{
		if (mConstrainedTable[i]) 
		{
			vec[i * 3 + 0] = 0;
			vec[i * 3 + 1] = 0;
			vec[i * 3 + 2] = 0;
		}

	}
}

void SoftBodyVertexConstraint::SetZeroColumnConstrainedDof(MatrixXd& mat) const
{
	CHECK(mObject);
	int numDofs = mObject->GetNumDofs();
	CHECK(numDofs == mat.rows());

	int numNodes = mObject->GetNumNodes();

	for (int i = 0; i < numNodes; ++i)
	{
		if (mConstrainedTable[i]) 
		{
			for (int j = 0; j < mat.cols(); ++j)
			{
				mat(3 * i + 0, j) = 0;
				mat(3 * i + 1, j) = 0;
				mat(3 * i + 2, j) = 0;
			}
		}

	}
}

void SoftBodyVertexConstraint::AddConstrainedDof(VectorXd& vec) const
{
	CHECK(mObject);
	int numDofs = vec.size();

	int numDofsComplete = mObject->GetNumDofs();
	CHECK(numDofs + 3 * static_cast<int>(mVertexIndices.size()) == numDofsComplete);

	VectorXd newVec(numDofsComplete);
	int numNodeComplete = mObject->GetNumNodes();
	int j = 0;
	for (int i = 0; i < numNodeComplete; ++i)
	{
		if (mConstrainedTable[i]) 
		{
			newVec[i * 3 + 0] = 0;
			newVec[i * 3 + 1] = 0;
			newVec[i * 3 + 2] = 0;

		}
		else
		{
			newVec[i * 3 + 0] = vec[j * 3 + 0];
			newVec[i * 3 + 1] = vec[j * 3 + 1];
			newVec[i * 3 + 2] = vec[j * 3 + 2];
			j++;
		}

	}
	vec = newVec;
}

void SoftBodyVertexConstraint::Serialize(DecoArchive& Ar) const
{
	Ar << mVertexIndices;
	Ar << mConstrainedTable;
}
void SoftBodyVertexConstraint::Deserialize(DecoArchive& Ar)
{
	Ar >> mVertexIndices;
	Ar >> mConstrainedTable;
}
