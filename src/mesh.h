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

using namespace std;

typedef Eigen::Matrix<float, Eigen::Dynamic,1> VectorX;
typedef Eigen::SparseMatrix<float> MatrixX;

struct Edge
{
    unsigned short  vertexIndex[2];
    unsigned short  triangleIndex[2];
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

class Mesh 
{
public:
	VectorX q;
	VectorX v;
	MatrixX M;
	MatrixX M_inv;

	float total_mass;
	
  vector<glm::vec4> vertices;
  vector<Edge> edges;
  vector<Triangle> triangles;
  vector<glm::vec3> normals;
  vector<GLushort> elements;
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

private:
  GLuint vbo_vertices, vbo_normals, ibo_elements;

};

#endif