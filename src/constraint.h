#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include "mesh.h"

typedef Eigen::Matrix<float, Eigen::Dynamic,1> VectorX;
typedef Eigen::Matrix<float, 3, 1, 0, 3 ,1> Vector3;
typedef Eigen::SparseMatrix<float> SpMat;
typedef Eigen::Triplet<float> T;

extern Mesh main_object;
enum {FIXEDPOINT, SPRING};

class Constraint
{
public:
	Constraint() {}
	~Constraint() {}

	int type;
};

class FixedPoint: public Constraint
{
public:
	void createS();
	void createAi();
	void createRHS();
	void createLHS();

	FixedPoint() {}
	FixedPoint(unsigned int index):vertexIndex(index),wi(10000.0){
		type = FIXEDPOINT;
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
	SpMat Bi;
	SpMat Ai_transpose;
	SpMat S;
	SpMat S_transpose;
	SpMat LHS;
	VectorX RHS;
	float wi;
};

class Spring: public Constraint
{
public:
	void createS();
	void createAi();
	void createRHS(const VectorX &q);
	void createLHS();

	Spring() {}
	Spring(unsigned int i0, unsigned int i1):wi(1000.0){
		type = SPRING;
		vertexIndex[0] = i0;
		vertexIndex[1] = i1;
		restLength = glm::length(glm::vec3(main_object.vertices[i0]) - glm::vec3(main_object.vertices[i1]));
		createS();
		createAi();
		createLHS();
	}
	~Spring(){}


	unsigned int vertexIndex[2];
	SpMat Ai;
	SpMat Bi;
	SpMat Ai_transpose;
	SpMat S;
	SpMat S_transpose;
	SpMat LHS;
	VectorX RHS;
	float wi;
	float restLength;

};

#endif