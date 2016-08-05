#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include "mesh.h"
#include "constraint.h"

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix;

extern Mesh main_object;
const float ground_height = -1.0;
const float EPSILON = 0.01;

std::vector<Constraint *> constraints;

Eigen::LLT<Matrix> lltOfM;

VectorX collide(const VectorX q){
	VectorX penetration(main_object.vertices.size() * 3);

	penetration.setZero();
	Vector3 n = Vector3(0.0,1.0,0.0);
	for(int i = 0; i < main_object.vertices.size(); i++){
		Vector3 qi = q.block_vec3(i);
		float dist = qi(1) - ground_height - EPSILON;
		if(dist < 0){
			penetration.block_vec3(i) += dist * n;
			Vector3 v = main_object.v.block_vec3(i);
			Vector3 vn = v.dot(n)/n.norm() * n;
			Vector3 vt = v - vn;
			//damp velocity components
			vn = -0.4 * vn;
			vt = 0.98 * vt;

			main_object.v.block_vec3(i) = vn + vt;
		}
	}
	return penetration;
}

void setupConstraints(){
	//set up fixed point constraints
	constraints.push_back(new FixedPoint(0));
	//set up spring constraints
	for(int i = 0; i < main_object.edges.size(); i++) {
		Edge edge = main_object.edges[i];
		constraints.push_back(new Spring(edge.vertexIndex[0],edge.vertexIndex[1]));
	}
}

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
	for(int i = 0; i < constraints.size(); i++) {
		Constraint *ci = constraints[i];
		if(ci->type == FIXEDPOINT){
			FixedPoint *fixedPoint = (FixedPoint *)ci;
			RHS += fixedPoint->RHS;
			LHS += fixedPoint->LHS;
		}
		else if (ci->type == SPRING) {
			Spring *spring = (Spring *)ci;
			spring->createRHS(q); 
			RHS += spring->RHS;
			LHS += spring->LHS;
		}
	}

/////////////////////////////////////////////////////////////////////
	lltOfM.compute(LHS);
	q_n = lltOfM.solve(RHS);

	VectorX v_n = (q_n - q)/h;
	main_object.q = q_n;
	main_object.v = v_n;

	// update mesh in openGL
	int num_vertices = main_object.vertices.size();
	//VectorX penetration = collide(main_object.q);
	//main_object.q -= penetration;

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

#endif