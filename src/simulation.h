#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include "mesh.h"

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix;

extern Mesh main_object;

Eigen::LLT<Matrix> lltOfM;

void update(int timestep) {
	glutTimerFunc(timestep, update, timestep);
	float h = (float)timestep / 100;
	VectorX q = main_object.q;
	VectorX q_n;
	VectorX v = main_object.v;
	SpMat M = main_object.M;
	SpMat M_inv = main_object.M_inv;

	VectorX fext(main_object.vertices.size() * 3);
	fext.setZero();
	const float gravity_g = 980;

	////////////////////////////////////////////////////////////////////////
	// choice of Ai and Bi 
	// from Shape-Up: Shaping Discrete Geometry with Projections eq.(5)
	SpMat Ai(6,6);
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
	SpMat Bi = Ai;
	float wi = 1.0;
  

	for(int i = 0; i < main_object.vertices.size(); i++) {
		fext[i * 3 + 1] = -gravity_g;
	}

	fext = M * fext;

	VectorX s = q + v * h + h * h * M_inv * fext ;
	q = s;
	VectorX p;
	VectorX RHS = (1.0 / (h * h)) * M * s;
	SpMat LHS = M / (h * h);

	// project on constraints
	for(int i = 0; i < main_object.edges.size(); i++) {
		Edge edge = main_object.edges[i];

		//from Position Based Dynamics 3.3 Constraint Projection
		p.resize(6);
		VectorX delta_p = q.block_vec3(edge.vertexIndex[0]) - q.block_vec3(edge.vertexIndex[1]);
		delta_p = (delta_p.norm() - edge.restLength) / 2 * delta_p.normalized();

		p.block_vec3(0) = q.block_vec3(edge.vertexIndex[0]) - delta_p;
		p.block_vec3(1) = q.block_vec3(edge.vertexIndex[1]) + delta_p;

		edge.createS(main_object.vertices.size());
		SpMat S = edge.S;
		SpMat S_transpose = S.transpose();
		SpMat RHSMatrix = wi * S_transpose * Ai * Bi;
		RHS += RHSMatrix * p;
		SpMat Ai_transpose = Ai.transpose();
		LHS += wi * S_transpose * Ai_transpose * Ai * S;
	}

	lltOfM.compute(LHS);
	q_n = lltOfM.solve(RHS);

	VectorX v_n = (q_n - q)/h;
	main_object.q = q_n;
	main_object.v = v_n;

	// update mesh in openGL
	int num_vertices = main_object.vertices.size();
	main_object.vertices.clear();
	for(int i = 0; i < num_vertices; i++) {
		VectorX v3 = main_object.q.block_vec3(i);
		glm::vec4 v4(v3(0,0),v3(1,0),v3(2,0),1.0);
		main_object.vertices.push_back(v4);
	}
	main_object.buildNormals();
	main_object.upload();
printf("one frame\n");
	//main_object.writeObj();
}

void intersect(const glm::vec4 &p){
	
}

#endif