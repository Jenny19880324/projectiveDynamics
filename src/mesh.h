#ifndef _MESH_H_
#define _MESH_H_

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

extern GLint attribute_v_coord;
extern GLint attribute_v_normal;
extern GLint uniform_m, uniform_v, uniform_p;
extern GLint uniform_m_3x3_inv_transp, uniform_v_inv;

typedef Eigen::Matrix<float, Eigen::Dynamic,1> VectorX;
typedef Eigen::SparseMatrix<float> SpMat;
typedef Eigen::Triplet<float> T;

#define block_vec3(a) block<3,1>(3*(a),0)


struct Edge
{
    unsigned short  vertexIndex[2];
    unsigned short  triangleIndex[2];
    float restLength;
    SpMat S;
    void createS(unsigned int num_vertices) {
      S.resize(6, 3 * num_vertices);
      std::vector<T> coefficients;
      coefficients.push_back(T(0, vertexIndex[0] * 3 + 0, 1));
      coefficients.push_back(T(1, vertexIndex[0] * 3 + 1, 1));
      coefficients.push_back(T(2, vertexIndex[0] * 3 + 2, 1));
      coefficients.push_back(T(3, vertexIndex[1] * 3 + 0, 1));
      coefficients.push_back(T(4, vertexIndex[1] * 3 + 1, 1));
      coefficients.push_back(T(5, vertexIndex[1] * 3 + 2, 1));
      S.setFromTriplets(coefficients.begin(), coefficients.end());
    }
};


struct Triangle
{
	unsigned short index[3];
	Triangle(unsigned short a, unsigned short b, unsigned short c){
		index[0] = a;
		index[1] = b;
		index[2] = c;
	}
};

//code from http://www.opengl-tutorial.org/beginners-tutorials/tutorial-7-model-loading/
class Mesh 
{
public:
	VectorX q;
	VectorX v;
	SpMat M;
	SpMat M_inv;

	float total_mass;
	
  std::vector<glm::vec4> vertices;
  std::vector<Edge> edges;
  std::vector<Triangle> triangles;
  std::vector<glm::vec3> normals;
  std::vector<GLushort> elements;
  glm::mat4 object2world;

  Mesh() : vbo_vertices(0), vbo_normals(0), ibo_elements(0), object2world(glm::mat4(1)), total_mass(1.0) {}
  ~Mesh() {
    if (vbo_vertices != 0)
      glDeleteBuffers(1, &vbo_vertices);
    if (vbo_normals != 0)
      glDeleteBuffers(1, &vbo_normals);
    if (ibo_elements != 0)
      glDeleteBuffers(1, &ibo_elements);
  }

  void upload();
  void draw();
  long buildEdges();
  void buildNormals();
  void writeObj();

private:
  GLuint vbo_vertices, vbo_normals, ibo_elements;

};

#endif