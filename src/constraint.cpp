#include "constraint.h"

void FixedPoint::createS(){
	S.resize(3, 3 * main_object.vertices.size());
	std::vector<T> coefficients;
 	coefficients.clear();
	coefficients.push_back(T(0, vertexIndex * 3 + 0, 1));
	coefficients.push_back(T(1, vertexIndex * 3 + 1, 1));
	coefficients.push_back(T(2, vertexIndex * 3 + 2, 1));
	S.setFromTriplets(coefficients.begin(), coefficients.end());
	S_transpose = S.transpose();

	Bi = Ai;
}

void FixedPoint::createAi() {
	Ai.resize(3,3);
	std::vector<T> coefficients;
	coefficients.clear();
	coefficients.push_back(T(0, 0, 1));
	coefficients.push_back(T(1, 1, 1));
	coefficients.push_back(T(2, 2, 1));
	Ai.setFromTriplets(coefficients.begin(), coefficients.end());
	Ai_transpose = Ai.transpose();
	Bi = Ai;
}

void FixedPoint::createRHS() {
	SpMat RHSMatrix = wi * S_transpose * Ai * Bi;
	RHS = RHSMatrix * fixedPosition;
}

void FixedPoint::createLHS() {
	LHS = wi * S_transpose * Ai_transpose * Ai * S;
}

void Spring::createS() {
	S.resize(6, 3 * main_object.vertices.size());
	std::vector<T> coefficients;
	coefficients.push_back(T(0, vertexIndex[0] * 3 + 0, 1));
	coefficients.push_back(T(1, vertexIndex[0] * 3 + 1, 1));
	coefficients.push_back(T(2, vertexIndex[0] * 3 + 2, 1));
	coefficients.push_back(T(3, vertexIndex[1] * 3 + 0, 1));
	coefficients.push_back(T(4, vertexIndex[1] * 3 + 1, 1));
	coefficients.push_back(T(5, vertexIndex[1] * 3 + 2, 1));
	S.setFromTriplets(coefficients.begin(), coefficients.end());
	S_transpose = S.transpose();
}

void Spring::createAi() {
	////////////////////////////////////////////////////////////////////////
	// choice of Ai and Bi 
	// from Shape-Up: Shaping Discrete Geometry with Projections eq.(5)
	Ai.resize(6,6);
	std::vector<T> coefficients;
	coefficients.push_back(T(0, 0, 0.5));
	coefficients.push_back(T(1, 1, 0.5));
	coefficients.push_back(T(2, 2, 0.5));
	coefficients.push_back(T(3, 3, 0.5));
	coefficients.push_back(T(4, 4, 0.5));
	coefficients.push_back(T(5, 5, 0.5));
	coefficients.push_back(T(0, 3, -0.5));
	coefficients.push_back(T(1, 4, -0.5));
	coefficients.push_back(T(2, 5, -0.5));
	coefficients.push_back(T(3, 0, -0.5));
	coefficients.push_back(T(4, 1, -0.5));
	coefficients.push_back(T(5, 2, -0.5));
	Ai.setFromTriplets(coefficients.begin(), coefficients.end());
	Ai_transpose = Ai.transpose();

	Bi = Ai;
}

void Spring::createRHS(const VectorX &q) {
	//from Position Based Dynamics 3.3 Constraint Projection
	VectorX p(6);
	VectorX delta_p = q.block_vec3(vertexIndex[0]) - q.block_vec3(vertexIndex[1]);
	delta_p = (delta_p.norm() - restLength) / 2 * delta_p.normalized();
	p.block_vec3(0) = q.block_vec3(vertexIndex[0]) - delta_p;
	p.block_vec3(1) = q.block_vec3(vertexIndex[1]) + delta_p;

	SpMat RHSMatrix = wi * S_transpose * Ai * Bi;
	RHS = RHSMatrix * p;
}

void Spring::createLHS() {
	LHS = wi * S_transpose * Ai_transpose * Ai * S;
}
