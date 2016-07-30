/**
 * From the OpenGL Programming wikibook: http://en.wikibooks.org/wiki/OpenGL_Programming
 * This file is in the public domain.
 * Contributors: Sylvain Beucler
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

/* Using the GLUT library for the base windowing setup */
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

/* GLM */
// #define GLM_MESSAGES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "shader_utils.h"
#include "mesh.h"
#include "mesh_utils.h"

int screen_width=800, screen_height=600;
bool compute_arcball;
int last_mx = 0, last_my = 0, cur_mx = 0, cur_my = 0;
int arcball_on = false;

GLint attribute_v_coord = -1;
GLint attribute_v_normal = -1;
GLint uniform_m = -1, uniform_v = -1, uniform_p = -1;
GLint uniform_m_3x3_inv_transp = -1, uniform_v_inv = -1;


using namespace std;

enum MODES { MODE_OBJECT, MODE_CAMERA, MODE_LIGHT, MODE_LAST } view_mode;
int rotY_direction = 0, rotX_direction = 0, transZ_direction = 0, strife = 0;
float speed_factor = 1;
glm::mat4 transforms[MODE_LAST];
int last_ticks = 0;

static unsigned int fps_start = glutGet(GLUT_ELAPSED_TIME);
static unsigned int fps_frames = 0;

Mesh ground, main_object, light_bbox;

void init_view() {
  main_object.object2world = glm::mat4(1);
  transforms[MODE_CAMERA] = glm::lookAt(
    glm::vec3(0.0,  0.0, 4.0),   // eye
    glm::vec3(0.0,  0.0, 0.0),   // direction
    glm::vec3(0.0,  1.0, 0.0));  // up
}

void onSpecial(int key, int x, int y) {
  int modifiers = glutGetModifiers();
  if ((modifiers & GLUT_ACTIVE_ALT) == GLUT_ACTIVE_ALT)
    strife = 1;
  else
    strife = 0;

  if ((modifiers & GLUT_ACTIVE_SHIFT) == GLUT_ACTIVE_SHIFT)
    speed_factor = 0.1;
  else
    speed_factor = 1;

  switch (key) {
  case GLUT_KEY_F1:
    view_mode = MODE_OBJECT;
    break;
  case GLUT_KEY_F2:
    view_mode = MODE_CAMERA;
    break;
  case GLUT_KEY_F3:
    view_mode = MODE_LIGHT;
    break;
  case GLUT_KEY_LEFT:
    rotY_direction = 1;
    break;
  case GLUT_KEY_RIGHT:
    rotY_direction = -1;
    break;
  case GLUT_KEY_UP:
    transZ_direction = 1;
    break;
  case GLUT_KEY_DOWN:
    transZ_direction = -1;
    break;
  case GLUT_KEY_PAGE_UP:
    rotX_direction = -1;
    break;
  case GLUT_KEY_PAGE_DOWN:
    rotX_direction = 1;
    break;
  case GLUT_KEY_HOME:
    init_view();
    break;
  }
}

void onSpecialUp(int key, int x, int y) {
  switch (key) {
  case GLUT_KEY_LEFT:
  case GLUT_KEY_RIGHT:
    rotY_direction = 0;
    break;
  case GLUT_KEY_UP:
  case GLUT_KEY_DOWN:
    transZ_direction = 0;
    break;
  case GLUT_KEY_PAGE_UP:
  case GLUT_KEY_PAGE_DOWN:
    rotX_direction = 0;
    break;
  }
}

/**
 * Get a normalized vector from the center of the virtual ball O to a
 * point P on the virtual ball surface, such that P is aligned on
 * screen's (X,Y) coordinates.  If (X,Y) is too far away from the
 * sphere, return the nearest point on the virtual ball surface.
 */
glm::vec3 get_arcball_vector(int x, int y) {
  glm::vec3 P = glm::vec3(1.0*x/screen_width*2 - 1.0,
              1.0*y/screen_height*2 - 1.0,
              0);
  P.y = -P.y;
  float OP_squared = P.x * P.x + P.y * P.y;
  if (OP_squared <= 1*1)
    P.z = sqrt(1*1 - OP_squared);  // Pythagore
  else
    P = glm::normalize(P);  // nearest point
  return P;
}

void logic() {
  /* FPS count */
  {
    fps_frames++;
    int delta_t = glutGet(GLUT_ELAPSED_TIME) - fps_start;
    if (delta_t > 1000) {
      cout << 1000.0 * fps_frames / delta_t << endl;
      fps_frames = 0;
      fps_start = glutGet(GLUT_ELAPSED_TIME);
    }
  }

  /* Handle keyboard-based transformations */
  int delta_t = glutGet(GLUT_ELAPSED_TIME) - last_ticks;
  last_ticks = glutGet(GLUT_ELAPSED_TIME);

  float delta_transZ = transZ_direction * delta_t / 1000.0 * 5 * speed_factor;  // 5 units per second
  float delta_transX = 0, delta_transY = 0, delta_rotY = 0, delta_rotX = 0;
  if (strife) {
    delta_transX = rotY_direction * delta_t / 1000.0 * 3 * speed_factor;  // 3 units per second
    delta_transY = rotX_direction * delta_t / 1000.0 * 3 * speed_factor;  // 3 units per second
  } else {
    delta_rotY =  rotY_direction * delta_t / 1000.0 * 120 * speed_factor;  // 120° per second
    delta_rotX = -rotX_direction * delta_t / 1000.0 * 120 * speed_factor;  // 120° per second
  }

  if (view_mode == MODE_OBJECT) {
    main_object.object2world = glm::rotate(main_object.object2world, glm::radians(delta_rotY), glm::vec3(0.0, 1.0, 0.0));
    main_object.object2world = glm::rotate(main_object.object2world, glm::radians(delta_rotX), glm::vec3(1.0, 0.0, 0.0));
    main_object.object2world = glm::translate(main_object.object2world, glm::vec3(0.0, 0.0, delta_transZ));
  } else if (view_mode == MODE_CAMERA) {
    // Camera is reverse-facing, so reverse Z translation and X rotation.
    // Plus, the View matrix is the inverse of the camera2world (it's
    // world->camera), so we'll reverse the transformations.
    // Alternatively, imagine that you transform the world, instead of positioning the camera.
    if (strife) {
      transforms[MODE_CAMERA] = glm::translate(glm::mat4(1.0), glm::vec3(delta_transX, 0.0, 0.0)) * transforms[MODE_CAMERA];
    } else {
      glm::vec3 y_axis_world = glm::mat3(transforms[MODE_CAMERA]) * glm::vec3(0.0, 1.0, 0.0);
      transforms[MODE_CAMERA] = glm::rotate(glm::mat4(1.0), glm::radians(-delta_rotY), y_axis_world) * transforms[MODE_CAMERA];
    }

    if (strife)
      transforms[MODE_CAMERA] = glm::translate(glm::mat4(1.0), glm::vec3(0.0, delta_transY, 0.0)) * transforms[MODE_CAMERA];
    else
      transforms[MODE_CAMERA] = glm::translate(glm::mat4(1.0), glm::vec3(0.0, 0.0, delta_transZ)) * transforms[MODE_CAMERA];

     transforms[MODE_CAMERA] = glm::rotate(glm::mat4(1.0), glm::radians(delta_rotX), glm::vec3(1.0, 0.0, 0.0)) * transforms[MODE_CAMERA];
  }

  /* Handle arcball */
  if (cur_mx != last_mx || cur_my != last_my) {
    glm::vec3 va = get_arcball_vector(last_mx, last_my);
    glm::vec3 vb = get_arcball_vector( cur_mx,  cur_my);
    float angle = acos(min(1.0f, glm::dot(va, vb)));
    glm::vec3 axis_in_camera_coord = glm::cross(va, vb);
    glm::mat3 camera2object = glm::inverse(glm::mat3(transforms[MODE_CAMERA]) * glm::mat3(main_object.object2world));
    glm::vec3 axis_in_object_coord = camera2object * axis_in_camera_coord;
    main_object.object2world = glm::rotate(main_object.object2world, angle, axis_in_object_coord);
    last_mx = cur_mx;
    last_my = cur_my;
  }

  // Model
  // Set in onDisplay() - cf. main_object.object2world

  // View
  glm::mat4 world2camera = transforms[MODE_CAMERA];

  // Projection
  glm::mat4 camera2screen = glm::perspective(45.0f, 1.0f*screen_width/screen_height, 0.1f, 100.0f);

  glUseProgram(program);
  glUniformMatrix4fv(uniform_v, 1, GL_FALSE, glm::value_ptr(world2camera));
  glUniformMatrix4fv(uniform_p, 1, GL_FALSE, glm::value_ptr(camera2screen));

  glm::mat4 v_inv = glm::inverse(world2camera);
  glUniformMatrix4fv(uniform_v_inv, 1, GL_FALSE, glm::value_ptr(v_inv));

  glutPostRedisplay();
}

void draw() {
  glClearColor(0.45, 0.45, 0.45, 1.0);
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  glUseProgram(program);

  main_object.draw();
  ground.draw();
}

void onDisplay()
{
  logic();
  draw();
  glutSwapBuffers();
}

void onMouse(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    arcball_on = true;
    last_mx = cur_mx = x;
    last_my = cur_my = y;
  } else {
    arcball_on = false;
  }
}

void onMotion(int x, int y) {
  if (arcball_on) {  // if left button is pressed
    cur_mx = x;
    cur_my = y;
  }
}

void onReshape(int width, int height) {
  screen_width = width;
  screen_height = height;
  glViewport(0, 0, screen_width, screen_height);
}

void free_resources()
{
  glDeleteProgram(program);
}

int main(int argc, char* argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA|GLUT_ALPHA|GLUT_DOUBLE|GLUT_DEPTH);
  glutInitWindowSize(screen_width, screen_height);
  glutCreateWindow("OBJ viewer");

  char* obj_filename = (char*) "../obj/suzanne.obj";
  char* v_shader_filename = (char*) "../glsl/phong-shading.v.glsl";
  char* f_shader_filename = (char*) "../glsl/phong-shading.f.glsl";
  if (argc != 4) {
    fprintf(stderr, "Usage: %s model.obj vertex_shader.v.glsl fragment_shader.f.glsl\n", argv[0]);
  } else {
    obj_filename = argv[1];
    v_shader_filename = argv[2];
    f_shader_filename = argv[3];
  }

  if (init_resources(obj_filename, v_shader_filename, f_shader_filename)) {
    init_view();
    glutDisplayFunc(onDisplay);
    glutSpecialFunc(onSpecial);
    glutSpecialUpFunc(onSpecialUp);
    glutMouseFunc(onMouse);
    glutMotionFunc(onMotion);
    glutReshapeFunc(onReshape);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LEQUAL);
    //glDepthRange(1, 0);
    last_ticks = glutGet(GLUT_ELAPSED_TIME);
    glutMainLoop();
  }

  free_resources();
  return 0;
}
