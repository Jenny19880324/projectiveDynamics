#include <algorithm>

void draw_sphere(float radius, int slices, int stacks) {
  Mesh sphere;
  //sphere.object2world = glm::mat4(1);
  float angle = glutGet(GLUT_ELAPSED_TIME) / 1000.0 * 45;  // 45° per second
  sphere.object2world = glm::rotate(glm::mat4(1), glm::radians(angle), glm::vec3(0,1,0))
    //* glm::translate(glm::mat4(1), glm::vec3(1,1,1))
    ;

  // for (float r = 0; r <= radius + 1e-6; r += radius / slices / 2) {
  //   for (float a = 0; a < 2*M_PI; a += 2*M_PI / stacks) {

  for (int i = 0; i < slices/2.0-1e-6; i++) {
    float r = radius*radius / (slices-1)*2 * i;  // -1 because 3 slices (top/middle/bottom) cut a sphere in 2
                                                 // *2 because we must have 100% of the radius at slices/2
                                                 // radius*radius in an attempt to linearly vary the square of the radius (x^2+y^2)
    r = sqrt(r);
    for (int j = 0; j < stacks; j++) {
      float a = 2*M_PI / stacks * j;
      float x = cos(a) * r;
      float y = sin(a) * r;
      float z = sqrt(max(0.0f, radius*radius - x*x - y*y));
      glm::vec4 v = glm::vec4(x,y,z,1);
      glm::vec3 n = glm::normalize(glm::vec3(x,y,z));
      sphere.vertices.push_back(v);
      sphere.normals.push_back(n);
      // cout << sphere.normals.size() - 1 << " ";
    }
    // cout << endl;
  }

  // Mirror the other half with -z, but in the same stack order
  // cout << ceil(slices/2.0) << endl;
  // cout << slices/2.0 << endl;
  for (int i = ceil(slices/2.0); i < slices; i++) {
    for (int j = 0; j < stacks; j++) {
      int idx = (slices-1-i) * stacks + j;
      glm::vec4 ov = sphere.vertices[idx];
      // cout << idx << " ";
      glm::vec4 v = glm::vec4(ov.x, ov.y, -ov.z, 1);
      glm::vec3 n = glm::normalize(glm::vec3(v.x,v.y,v.z));
      sphere.vertices.push_back(v);
      sphere.normals.push_back(n);
    }
    // cout << endl;
  }

  for (int i = 0; i < slices-1; i++) {
    for (int j = 0; j < stacks; j++) {
      sphere.elements.push_back(   i  * stacks + j);
      sphere.elements.push_back((i+1) * stacks + j);
      sphere.elements.push_back(   i  * stacks + (j+1)%stacks);
      sphere.elements.push_back(   i  * stacks + (j+1)%stacks);
      sphere.elements.push_back((i+1) * stacks + j);
      sphere.elements.push_back((i+1) * stacks + (j+1)%stacks);
    }
  }
  
  // static int i = 450;
  // i+=1;
  // if (i == sphere.elements.size())
  //   i = 0;
  // cout << sphere.vertices.size() << endl;
  // cout << *max_element(sphere.elements.begin(), sphere.elements.end()) << endl;
  // sphere.elements.resize(i);
  sphere.upload();


  // glEnableVertexAttribArray(attribute_v_coord);
  // glBindBuffer(GL_ARRAY_BUFFER, sphere.vbo_vertices);
  // glVertexAttribPointer(
  //   attribute_v_coord,  // attribute
  //   4,                  // number of elements per vertex, here (x,y,z,w)
  //   GL_FLOAT,           // the type of each element
  //   GL_FALSE,           // take our values as-is
  //   0,                  // no extra data between each position
  //   0                   // offset of first element
  // );
  // int size;  glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &size);
  // cout << size << endl;

  /* Apply object's transformation matrix */
  glUniformMatrix4fv(uniform_m, 1, GL_FALSE, glm::value_ptr(glm::mat4(sphere.object2world)));
  glm::mat3 m_3x3_inv_transp = glm::transpose(glm::inverse(glm::mat3(sphere.object2world)));
  glUniformMatrix3fv(uniform_m_3x3_inv_transp, 1, GL_FALSE, glm::value_ptr(m_3x3_inv_transp));

  // static int i = 0;
  // i++;
  // if (i == sphere.elements.size())
  //   i = 0;
  // //glDrawArrays(GL_LINE_STRIP, 0, i);
  // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphere.ibo_elements);
  // glDrawElements(GL_TRIANGLES,
  // 		 sphere.elements.size()
  // 		 //i
  // 		 , GL_UNSIGNED_SHORT, 0);
  // //cout << sphere.vertices.size() << endl;
  // glDisableVertexAttribArray(attribute_v_coord);

  sphere.draw();
  glLineWidth(2);
  sphere.draw_bbox();
}
