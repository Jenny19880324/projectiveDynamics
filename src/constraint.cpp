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
}

void FixedPoint::createAi() {
	Ai.resize(3,3);
	std::vector<T> coefficients;
	coefficients.clear();
	coefficients.push_back(T(0, 0, 1));
	coefficients.push_back(T(1, 1, 1));
	coefficients.push_back(T(2, 2, 1));
	Ai.setFromTriplets(coefficients.begin(), coefficients.end());
}

void FixedPoint::createRHS() {
	SpMat Bi = Ai;
	SpMat RHSMatrix = wi * S_transpose * Ai * Bi;
	RHS = RHSMatrix * fixedPosition;
}

void FixedPoint::createLHS() {
	SpMat Ai_transpose = Ai.transpose();
	LHS = wi * S_transpose * Ai_transpose * Ai * S;
}
