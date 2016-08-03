#ifndef _MESH_UTILS_H_
#define _MESH_UTILS_H_

#include <vector>
#include <glm/glm.hpp>

#include "shader_utils.h"
#include "mesh.h"

#define GROUND_SIZE 20

extern Mesh ground;
extern Mesh main_object;
extern Mesh light_bbox;

GLuint program;
extern GLint attribute_v_coord;
extern GLint attribute_v_normal;
extern GLint uniform_m, uniform_v, uniform_p;
extern GLint uniform_m_3x3_inv_transp, uniform_v_inv;

using namespace std;

void load_obj(const char* filename, Mesh* mesh) {
  ifstream in(filename, ios::in);
  if (!in) { cerr << "Cannot open " << filename << endl; exit(1); }
  
  string line;
  while (getline(in, line)) {
    if (line.substr(0,2) == "v ") {
      istringstream s(line.substr(2));
      glm::vec4 v; s >> v.x; s >> v.y; s >> v.z; v.w = 1.0;
      mesh->vertices.push_back(v);
    }  else if (line.substr(0,2) == "f ") {
      istringstream s(line.substr(2));
      GLushort a,b,c;
      s >> a; s >> b; s >> c;
      a--; b--; c--;
      mesh->elements.push_back(a); mesh->elements.push_back(b); mesh->elements.push_back(c);
      mesh->triangles.push_back(Triangle(a,b,c));
    }
    else if (line[0] == '#') { /* ignoring this line */ }
    else { /* ignoring this line */ }
  }

  // assign initial position to q
  mesh->q.resize(mesh->vertices.size() * 3);
  for(int i = 0; i < mesh->vertices.size(); i++) {
  	glm::vec4 v = mesh->vertices[i];
  	Eigen::Array3f q;
	q << v.x, v.y, v.z;
	mesh->q.block<3,1>(3 * i,0) = q;
  }

  // assign initial velocity to v
  mesh->v.resize(mesh->vertices.size() * 3);
  mesh->v.setZero();

  // assign unit mass
  float unit_mass = mesh -> total_mass / mesh->vertices.size();
  float unit_mass_inv = 1.0 / unit_mass;

	std::vector<Eigen::Triplet<float,int> > m_triplets;
	std::vector<Eigen::Triplet<float,int> > m_inv_triplets;
  	for (int i = 0; i < mesh->vertices.size() * 3; i++)
	{
		m_triplets.push_back(Eigen::Triplet<float,int>(i, i, unit_mass));
		m_inv_triplets.push_back(Eigen::Triplet<float,int>(i, i, unit_mass_inv));
	}
	mesh->M.resize(mesh->vertices.size() * 3, mesh->vertices.size() * 3);
	mesh->M_inv.resize(mesh->vertices.size() * 3, mesh->vertices.size() * 3);
	mesh->M.setFromTriplets(m_triplets.begin(), m_triplets.end());
	mesh->M_inv.setFromTriplets(m_inv_triplets.begin(), m_inv_triplets.end());
	m_triplets.clear();
	m_inv_triplets.clear();

  // build edge list
  mesh->buildEdges();

  //normals
  mesh->buildNormals();
}

int init_resources(char* model_filename, char* vshader_filename, char* fshader_filename)
{
  load_obj(model_filename, &main_object);
  // mesh position initialized in init_view()

  for (int i = -GROUND_SIZE/2; i < GROUND_SIZE/2; i++) {
    for (int j = -GROUND_SIZE/2; j < GROUND_SIZE/2; j++) {
      ground.vertices.push_back(glm::vec4(i,   0.0,  j+1, 1.0));
      ground.vertices.push_back(glm::vec4(i+1, 0.0,  j+1, 1.0));
      ground.vertices.push_back(glm::vec4(i,   0.0,  j,   1.0));
      ground.vertices.push_back(glm::vec4(i,   0.0,  j,   1.0));
      ground.vertices.push_back(glm::vec4(i+1, 0.0,  j+1, 1.0));
      ground.vertices.push_back(glm::vec4(i+1, 0.0,  j,   1.0));
      for (unsigned int k = 0; k < 6; k++)
    ground.normals.push_back(glm::vec3(0.0, 1.0, 0.0));
    }
  }

  glm::vec3 light_position = glm::vec3(0.0,  1.0,  2.0);
  light_bbox.vertices.push_back(glm::vec4(-0.1, -0.1, -0.1, 0.0));
  light_bbox.vertices.push_back(glm::vec4( 0.1, -0.1, -0.1, 0.0));
  light_bbox.vertices.push_back(glm::vec4( 0.1,  0.1, -0.1, 0.0));
  light_bbox.vertices.push_back(glm::vec4(-0.1,  0.1, -0.1, 0.0));
  light_bbox.vertices.push_back(glm::vec4(-0.1, -0.1,  0.1, 0.0));
  light_bbox.vertices.push_back(glm::vec4( 0.1, -0.1,  0.1, 0.0));
  light_bbox.vertices.push_back(glm::vec4( 0.1,  0.1,  0.1, 0.0));
  light_bbox.vertices.push_back(glm::vec4(-0.1,  0.1,  0.1, 0.0));
  light_bbox.object2world = glm::translate(glm::mat4(1), light_position);

  main_object.upload();
  ground.upload();
  light_bbox.upload();


  /* Compile and link shaders */
  GLint link_ok = GL_FALSE;
  GLint validate_ok = GL_FALSE;

  GLuint vs, fs;
  if ((vs = create_shader(vshader_filename, GL_VERTEX_SHADER))   == 0) return 0;
  if ((fs = create_shader(fshader_filename, GL_FRAGMENT_SHADER)) == 0) return 0;

  program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
  if (!link_ok) {
    fprintf(stderr, "glLinkProgram:");
    print_log(program);
    return 0;
  }
  glValidateProgram(program);
  glGetProgramiv(program, GL_VALIDATE_STATUS, &validate_ok);
  if (!validate_ok) {
    fprintf(stderr, "glValidateProgram:");
    print_log(program);
  }

  const char* attribute_name;
  attribute_name = "v_coord";
  attribute_v_coord = glGetAttribLocation(program, attribute_name);
  if (attribute_v_coord == -1) {
    fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
    return 0;
  }
  attribute_name = "v_normal";
  attribute_v_normal = glGetAttribLocation(program, attribute_name);
  if (attribute_v_normal == -1) {
    fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
    return 0;
  }
  const char* uniform_name;
  uniform_name = "m";
  uniform_m = glGetUniformLocation(program, uniform_name);
  if (uniform_m == -1) {
    fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
    return 0;
  }
  uniform_name = "v";
  uniform_v = glGetUniformLocation(program, uniform_name);
  if (uniform_v == -1) {
    fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
    return 0;
  }
  uniform_name = "p";
  uniform_p = glGetUniformLocation(program, uniform_name);
  if (uniform_p == -1) {
    fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
    return 0;
  }
  uniform_name = "m_3x3_inv_transp";
  uniform_m_3x3_inv_transp = glGetUniformLocation(program, uniform_name);
  if (uniform_m_3x3_inv_transp == -1) {
    fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
    return 0;
  }
  uniform_name = "v_inv";
  uniform_v_inv = glGetUniformLocation(program, uniform_name);
  if (uniform_v_inv == -1) {
    fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
    return 0;
  }

  //fps_start = glutGet(GLUT_ELAPSED_TIME);

  return 1;
}


#endif