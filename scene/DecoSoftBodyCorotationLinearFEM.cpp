#include "DecoSoftBodyCorotationLinearFEM.h"
#include "utility/polarDecomposition.h"
#include "ForceRecorder.h"

DecoSoftObjectCorotationLinearFEM::DecoSoftObjectCorotationLinearFEM()
{

}

DecoSoftObjectCorotationLinearFEM::~DecoSoftObjectCorotationLinearFEM()
{

}

const BlockSparseMatrix& DecoSoftObjectCorotationLinearFEM::GetDampingMatrix()
{
	mDampingMatrix = mDampingConstant1 * GetMassMatrix() + mDampingConstant2 * GetDfDx();
	mDampingMatrix.UpdateCSR();
	return mDampingMatrix;
}

const BlockSparseMatrix& DecoSoftObjectCorotationLinearFEM::GetAugmentedMassMatrix(double timeStep)
{
	mAugmentedMassMatrix = mMassMatrix + timeStep * GetDampingMatrix() + timeStep * timeStep * GetDfDx();
	mAugmentedMassMatrix.UpdateCSR();
	return mAugmentedMassMatrix;
}

const BlockSparseMatrix& DecoSoftObjectCorotationLinearFEM::GetDfDx()
{
	mK.UpdateCSR();
	return mK;
}

#if LLT_SOLVE
BlockSparseMatrix& DecoSoftObjectCorotationLinearFEM::GetReferenceMassMatrix()
{
	mReferenceMassForImplicitEuler.UpdateLLT();
	return mReferenceMassForImplicitEuler;
}
#endif

const BlockSparseMatrix& DecoSoftObjectCorotationLinearFEM::GetMk()
{
	mK.UpdateCSR();
	return mK;
}
const BlockSparseMatrix& DecoSoftObjectCorotationLinearFEM::GetMf()
{
	mF.UpdateCSR();
	return mF;
}
const VectorXd& DecoSoftObjectCorotationLinearFEM::GetMx() const
{
	return mX;
}

void DecoSoftObjectCorotationLinearFEM::planInternalForce()
{
	int numTets = mDeformedShape->GetNumTetrahedra();
	for (int i = 0; i < numTets; ++i)
	{
		Tetrahedron tet = mDeformedShape->GetIthTetrahedron(i);
		Tetrahedron tetOriginal = mRestShape->GetIthTetrahedron(i);
		double originalVolume = tetOriginal.CalculateSignVolume();
		double deformedVolume = tet.CalculateSignVolume();
		if (originalVolume * deformedVolume <= 0)
		{
			printf("Warning!!! %dth tetrahedron is inverted\n", i);
		}

		matrix33 P;
		P[0] = tet.mVertices[1] - tet.mVertices[0];
		P[1] = tet.mVertices[2] - tet.mVertices[0];
		P[2] = tet.mVertices[3] - tet.mVertices[0];
		P = P * mRestShapeInvertedX[i];

		double* A = P.getRowMajor();
		double Q[9], S[9];
		PolarDecomposition::DoPolarDecomposition(A, Q, S);
		matrix33 R;

		R.setRowMajor(Q);

		delete[] A;

		double scale = 1;
		vector3 tetCenter = tetOriginal.CalculateCenter();
		scale *= tetCenter.y;

		matrix33 internalStressLocal(vector3(1, 0, 0), vector3(0, 0, 0), vector3(0, 0, 0));
		internalStressLocal *= scale;
		matrix33 stress = R * internalStressLocal * R.transpose();


		for (int ithFace = 0; ithFace < 4; ++ithFace)
		{
			Vector3d fFaceEigenFormat = fromStressToForce(stress, tet, ithFace);
			vector3 fFace(fFaceEigenFormat[0], fFaceEigenFormat[1], fFaceEigenFormat[2]);
			mNodes[tet.mVertIndices[ithFace]].mForceAccumulator += fFace / 3.0;
			mNodes[tet.mVertIndices[(ithFace + 1) % 4]].mForceAccumulator += fFace / 3.0;
			mNodes[tet.mVertIndices[(ithFace + 2) % 4]].mForceAccumulator += fFace / 3.0;
		}
	}
}



void DecoSoftObjectCorotationLinearFEM::precomputation()
{
	DecoSoftObject::precomputation();
	int numVertices = mRestShape->GetNumVertices();
	formatPositionToEigenVector(*mRestShape, mX);

	mK.SetBlockInfo(3, 3, numVertices, numVertices);
	mK.SetToZero();

	mF.SetBlockInfo(3, 3, numVertices, numVertices);
	mF.SetToZero();
	mKPerTet.clear();

	mReferenceMassForImplicitEuler.SetBlockInfo(3, 3, numVertices, numVertices);
	mReferenceMassForImplicitEuler.SetToZero();


	int numTets = mRestShape->GetNumTetrahedra();
	mLastDeformationGradient.resize(numTets);
	for (int i = 0; i < numTets; ++i)
	{
		mLastDeformationGradient[i] = IdentityMatrix33();
		Tetrahedron tet = mRestShape->GetIthTetrahedron(i);
		matrix33 X;
		X[0] = tet.mVertices[1] - tet.mVertices[0];
		X[1] = tet.mVertices[2] - tet.mVertices[0];
		X[2] = tet.mVertices[3] - tet.mVertices[0];
		matrix33 invX = mRestShapeInvertedX[i];
		double V = X.determinant();
		double a = V * mYoungsModulus * (1 - mPoissonRatio) / (1 + mPoissonRatio) / (1 - 2 * mPoissonRatio);
		double b = V * mYoungsModulus * mPoissonRatio  / (1 + mPoissonRatio) / (1 - 2 * mPoissonRatio);
		double c = V * mYoungsModulus * (1 - 2 * mPoissonRatio) / (1 + mPoissonRatio) / (1 - 2 * mPoissonRatio);
		vector3 y[4];
		y[1] = vector3(invX[0][0], invX[1][0], invX[2][0]);
		y[2] = vector3(invX[0][1], invX[1][1], invX[2][1]);
		y[3] = vector3(invX[0][2], invX[1][2], invX[2][2]);
		y[0] = -y[1] - y[2] - y[3];

		Eigen::Matrix<double, 12, 12> kPerTet;
		kPerTet = Eigen::Matrix<double, 12, 12>::Zero();

		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{

				Eigen::Matrix3d mat1, mat2, mat3, mat4, mat5, mat6;

				mat1 << y[i].x, 0, 0, 
					    0, y[i].y, 0,
						0, 0, y[i].z;

				mat2 << a, b, b,
					    b, a, b, 
						b, b, a; 

				mat3 << y[j].x, 0, 0,
				        0, y[j].y, 0, 
						0, 0, y[j].z;

                mat4 << y[i].y, 0, y[i].z,
					    y[i].x, y[i].z, 0,
						0, y[i].y, y[i].x;

				mat5 << c, 0, 0,
				        0, c, 0, 
						0, 0, c;

				mat6 << y[j].y, y[j].x, 0,
                        0, y[j].z, y[j].y,
						y[j].z, 0, y[j].x;

				Eigen::Matrix3d Kij = mat1 * mat2 * mat3 + mat4 * mat5 * mat6; 

				int indexi = tet.mVertIndices[i];
				int indexj = tet.mVertIndices[j];
				kPerTet.block<3, 3>(3 * i, 3 * j) = Kij;

				mK.AddBlockMatrixTo(indexi, indexj, Kij);


			}
		}
		mKPerTet.push_back(kPerTet);
	}
	mF = mK;
#if LLT_SOLVE
	mReferenceMassForImplicitEuler = mK;
	mReferenceMassForImplicitEuler *= (timeStep * timeStep);
	mReferenceMassForImplicitEuler += mMassMatrix;
	mReferenceMassForImplicitEuler.UpdateLLT();
#endif
	mD = MatrixXd::Zero(6, 6);
	mD(0, 0) = mD(1, 1) = mD(2, 2) = 1.0 - mPoissonRatio;
	mD(3, 3) = mD(4, 4) = mD(5, 5) = 1.0 - 2 * mPoissonRatio;
	mD(0, 1) = mD(1, 0) = mD(0, 2) = mD(2, 0) = mD(1, 2) = mD(2, 1) = mPoissonRatio;
	mD *= mYoungsModulus / (1 + mPoissonRatio) / (1 - 2 * mPoissonRatio);

	
	mYoungsRatioPerTet.resize(numTets);
	for (int i = 0; i < numTets; ++i)
	{
		Tetrahedron tet = mRestShape->GetIthTetrahedron(i);
		double scale = 1.0 / (tet.mVertices[0].y * tet.mVertices[0].y + 0.5);
		mYoungsRatioPerTet[i] = 1.0;
	}
}

bool DecoSoftObjectCorotationLinearFEM::canbeLazy()
{
	mNotLazyTetIdx.clear();
    int numTets = mRestShape->GetNumTetrahedra();
	bool bNeedUpdate = false;

    for (int ithTet = 0; ithTet < numTets; ++ithTet)
    {
        Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
        matrix33 P;
        P[0] = tet.mVertices[1] - tet.mVertices[0];
        P[1] = tet.mVertices[2] - tet.mVertices[0];
        P[2] = tet.mVertices[3] - tet.mVertices[0];
        P = P * mRestShapeInvertedX[ithTet];

        matrix33 diff = P - mLastDeformationGradient[ithTet];
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                if (abs(diff[i][j]) > 0.0)
				{
                    bNeedUpdate = true;
					break;
				}
            }
        }

    }
	if (bNeedUpdate)
	{
		for (int ithTet = 0; ithTet < numTets; ++ithTet)
			mNotLazyTetIdx.push_back(ithTet);
	}

    return mNotLazyTetIdx.empty();
}

void DecoSoftObjectCorotationLinearFEM::assembleK(double timeStep)
{

    if (canbeLazy())
    {
        return;
    }

	int numTets = mRestShape->GetNumTetrahedra();
	int numVertices = mRestShape->GetNumVertices();
	
	mK.SetToZero();
	mF.SetToZero();

	calculateKPerTet();

	int numNotLazyTets = static_cast<int>(mNotLazyTetIdx.size());
	//LOG(INFO) << numNotLazyTets << " out of " << numTets << " are updated.";
#if LLT_SOLVE
	mRefMatrixChangedColumns.clear();
#endif
	for (int ithIdx = 0; ithIdx < numNotLazyTets; ++ithIdx)
	//for (int ithTet = 0; ithTet < numTets; ++ithTet)
	{
		int ithTet = mNotLazyTetIdx[ithIdx];
		Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
		matrix33 P;
		P[0] = tet.mVertices[1] - tet.mVertices[0];
		P[1] = tet.mVertices[2] - tet.mVertices[0];
		P[2] = tet.mVertices[3] - tet.mVertices[0];
		P = P * mRestShapeInvertedX[ithTet];
		matrix33 lastP = mLastDeformationGradient[ithTet];

        mLastDeformationGradient[ithTet] = P;
        Matrix3d R, S;
        Matrix3d transformation, lastTransformation;
        transformation << P[0][0], P[1][0], P[2][0],
                          P[0][1], P[1][1], P[2][1],
                          P[0][2], P[1][2], P[2][2];
        PolarDecomposition::DoPolarDecomposition(transformation, R, S);
//		R = Matrix3d::Identity();
		Eigen::Matrix<double, 12, 12> RPerTet = Eigen::Matrix<double, 12, 12>::Zero();
		for (int i = 0; i < 4; ++i)
		{
			RPerTet.block<3, 3>(3 * i, 3 * i) = R;
		}
		Eigen::MatrixXd RKPerTet = RPerTet * mKPerTet[ithTet];
		Eigen::MatrixXd RKRTransPerTet = RKPerTet * RPerTet.transpose();
		Eigen::MatrixXd KRPerTet = mKPerTet[ithTet] * RPerTet;
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				int indexi = tet.mVertIndices[i];
				int indexj = tet.mVertIndices[j];

				//mK.AddBlockMatrixTo(indexi, indexj, RKRTransPerTet.block<3,3>(3 * i, 3 * j));
				//mF.AddBlockMatrixTo(indexi, indexj, RKPerTet.block<3,3>(3 * i, 3 * j));

				mK.AddBlockMatrixTo(indexi, indexj, mKPerTet[ithTet].block<3,3>(3 * i, 3 * j));
				mF.AddBlockMatrixTo(indexi, indexj, KRPerTet.block<3,3>(3 * i, 3 * j));
#if LLT_SOLVE
				mRefMatrixChangedColumns.insert(indexj * 3 + 0);
				mRefMatrixChangedColumns.insert(indexj * 3 + 1);
				mRefMatrixChangedColumns.insert(indexj * 3 + 2);
#endif
			}
		}
	}
	
#if LLT_SOLVE
	updateReferenceMassMatrix(timeStep);
#endif
}

#if LLT_SOLVE
void DecoSoftObjectCorotationLinearFEM::updateReferenceMassMatrix(double timeStep)
{
	const BlockSparseMatrix& dfdx = GetDfDx();
	const BlockSparseMatrix& mass = GetMassMatrix();	
	BlockSparseMatrix newReferenceMassMatrix = dfdx;
	newReferenceMassMatrix *= (timeStep * timeStep);
	newReferenceMassMatrix += mass;

	mReferenceMassForImplicitEuler.IncrementalUpdateTo(newReferenceMassMatrix, mRefMatrixChangedColumns);
}
#endif

void DecoSoftObjectCorotationLinearFEM::accumulateForce(double timeStep)
{
	if (mbNeedRecalculateForce)
	{
		int numNodes = static_cast<int>(mNodes.size());

		VectorXd gravity = CalculateGravityForce(timeStep);
		if (strstr(name.c_str(), "ground") == NULL) //for the ground plane
		{
			for (int i = 0; i < numNodes; ++i)
			{
				mNodes[i].mForceAccumulator += vector3(gravity[3 * i + 0], gravity[3 * i + 1], gravity[3 * i + 2]);
				mForces->mGravityForce[i] = vector3(gravity[3 * i + 0], gravity[3 * i + 1], gravity[3 * i + 2]);
			}
		}
		

		Eigen::VectorXd internalForce = CalculateElasticForce(timeStep);


		for (int i = 0; i < numNodes; ++i)
		{
			mNodes[i].mForceAccumulator += vector3(internalForce[3 * i], internalForce[3 * i + 1], internalForce[3 * i + 2]);
			mForces->mElasticForce[i] = vector3(internalForce[3 * i + 0], internalForce[3 * i + 1], internalForce[3 * i + 2]);
		}
		//planInternalForce();
		mbNeedRecalculateForce = false;
	}

}

Eigen::VectorXd DecoSoftObjectCorotationLinearFEM::CalculateElasticStress(double timeStep)
{
	int numTets = mDeformedShape->GetNumTetrahedra();

	Eigen::VectorXd stress = Eigen::VectorXd::Zero(6 * numTets);
	return stress;

}



Eigen::VectorXd DecoSoftObjectCorotationLinearFEM::CalculateElasticForce(double timeStep)
{
	assembleK(timeStep);
	formatPositionToEigenVector(*mDeformedShape, mP);
	mK.UpdateCSR();
	mF.UpdateCSR();

	Eigen::VectorXd internalForce = mK * mP - mF * mX;

	return -internalForce;
}



MatrixXd DecoSoftObjectCorotationLinearFEM::calculateDStrainDx(int ithTet)
{
    const int numVPerTet = 4;
    const int numComponentsPerV = 3;

    MatrixXd bPerTet = MatrixXd::Zero(6, 12);

	Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
	
	double sixV = 6 * tet.CalculateSignVolume();
    for (int ithVertex = 0; ithVertex < numVPerTet; ++ithVertex)
    {
		vector3 vb = tet.mVertices[(ithVertex + 1) % numVPerTet];
		vector3 vc = tet.mVertices[(ithVertex + 2) % numVPerTet];
		vector3 vd = tet.mVertices[(ithVertex + 3) % numVPerTet];
		Matrix3d bMat;
		bMat << 1,    1,    1, 
				vb.y, vc.y, vd.y,
				vb.z, vc.z, vd.z;
		Matrix3d cMat;
		cMat << 1,    1,    1, 
				vb.x, vc.x, vd.x,
				vb.z, vc.z, vd.z;
		Matrix3d dMat;
		dMat << 1,    1,    1, 
				vb.x, vc.x, vd.x,
				vb.y, vc.y, vd.y;

		double bi = powf(-1, ithVertex + 1) * bMat.determinant();
		double ci = powf(-1, ithVertex) * cMat.determinant();
		double di = powf(-1, ithVertex + 1) * dMat.determinant();

		bPerTet(0, ithVertex * numComponentsPerV + 0) = bi;
		bPerTet(1, ithVertex * numComponentsPerV + 1) = ci;
		bPerTet(2, ithVertex * numComponentsPerV + 2) = di;

		bPerTet(3, ithVertex * numComponentsPerV + 0) = ci;
		bPerTet(3, ithVertex * numComponentsPerV + 1) = bi;

		bPerTet(4, ithVertex * numComponentsPerV + 1) = di;
		bPerTet(4, ithVertex * numComponentsPerV + 2) = ci;

		bPerTet(5, ithVertex * numComponentsPerV + 0) = di;
		bPerTet(5, ithVertex * numComponentsPerV + 2) = bi;
    }
	VectorXd testForce = VectorXd::Zero(6);
	
	for (int i = 0; i < 12; ++i)
	{
		testForce += bPerTet.col(i);
	}
	CHECK_NEAR(testForce.norm(), 0, EPSILON_FLOAT);

	for (int ithRow = 0; ithRow < 6; ++ithRow)
	{
		VectorXd testTorque = VectorXd::Zero(3);

		for (int i = 0; i < numVPerTet; ++i)
		{
			Vector3d x = tet.mVertices[i].ConvertToEigen();
			Vector3d row = bPerTet.block<1, 3>(ithRow, 3 * i).transpose();
			testTorque += x.cross(row);
		}
		CHECK_NEAR(testTorque.norm(), 0, EPSILON_FLOAT);
	}

	//bPerTet /= sixV;

	return bPerTet;
}

void DecoSoftObjectCorotationLinearFEM::calculateBPerTet()
{
    int numTets = mDeformedShape->GetNumTetrahedra();
    mBPerTet.resize(numTets);
    
    for (int ithTet = 0; ithTet < numTets; ++ithTet)
    {
        MatrixXd bPerTet = calculateDStrainDx(ithTet);
        mBPerTet[ithTet] = bPerTet;
    }
}

void DecoSoftObjectCorotationLinearFEM::calculateKPerTet()
{
	calculateBPerTet();
	int numTets = mDeformedShape->GetNumTetrahedra();

	mKPerTet.resize(numTets);

	
	for (int ithTet = 0; ithTet < numTets; ++ithTet)
	{
		Tetrahedron tet = mDeformedShape->GetIthTetrahedron(ithTet);
		double sixV = 6 * tet.CalculateSignVolume();

		mKPerTet[ithTet] = 1.0 / sixV * mBPerTet[ithTet].transpose() * mD * mBPerTet[ithTet] * mYoungsRatioPerTet[ithTet];

	}

}


void DecoSoftObjectCorotationLinearFEM::formatPositionToEigenVector(const TetMesh& mesh, Eigen::VectorXd& vector)
{
	int numVertices = mesh.GetNumVertices();
	vector = Eigen::VectorXd(3 * numVertices);
	for (int i = 0; i < numVertices; ++i)
	{
		vector(3 * i + 0) = mesh.GetIthVertex(i).x;
		vector(3 * i + 1) = mesh.GetIthVertex(i).y;
		vector(3 * i + 2) = mesh.GetIthVertex(i).z;
	}
}