#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include "mesh.h"

typedef Eigen::Matrix<float, Eigen::Dynamic,1> VectorX;
typedef Eigen::Matrix<float, 3, 1, 0, 3 ,1> Vector3;
typedef Eigen::SparseMatrix<float> SpMat;
typedef Eigen::Triplet<float> T;

extern Mesh main_object;

class FixedPoint
{
public:
	void createS();
	void createAi();
	void createRHS();
	void createLHS();

	FixedPoint() {}
	FixedPoint(unsigned int index):vertexIndex(index),wi(1.0){
		fixedPosition << main_object.vertices[index].x, main_object.vertices[index].y, main_object.vertices[index].z;
		createS();
		createAi();
		createRHS();
		createLHS();
	}
	~FixedPoint(){}


	Vector3 fixedPosition;
	unsigned int vertexIndex;
	SpMat Ai;
	SpMat S;
	SpMat S_transpose;
	SpMat LHS;
	VectorX RHS;
	float wi;
};

#endif