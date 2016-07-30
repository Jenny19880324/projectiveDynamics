#include "mesh.h"

/**
* Store object vertices, normals and/or elements in graphic card
* buffers
*/
void Mesh::upload() {
    if (this->vertices.size() > 0) {
      glGenBuffers(1, &this->vbo_vertices);
      glBindBuffer(GL_ARRAY_BUFFER, this->vbo_vertices);
      glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(this->vertices[0]),
           this->vertices.data(), GL_STATIC_DRAW);
    }

    if (this->normals.size() > 0) {
      glGenBuffers(1, &this->vbo_normals);
      glBindBuffer(GL_ARRAY_BUFFER, this->vbo_normals);
      glBufferData(GL_ARRAY_BUFFER, this->normals.size() * sizeof(this->normals[0]),
           this->normals.data(), GL_STATIC_DRAW);
    }

    if (this->elements.size() > 0) {
      glGenBuffers(1, &this->ibo_elements);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->ibo_elements);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->elements.size() * sizeof(this->elements[0]),
           this->elements.data(), GL_STATIC_DRAW);
    }
  }

/**
* Draw the object
*/
void Mesh::draw() {
    if (this->vbo_vertices != 0) {
      glEnableVertexAttribArray(attribute_v_coord);
      glBindBuffer(GL_ARRAY_BUFFER, this->vbo_vertices);
      glVertexAttribPointer(
        attribute_v_coord,  // attribute
        4,                  // number of elements per vertex, here (x,y,z,w)
        GL_FLOAT,           // the type of each element
        GL_FALSE,           // take our values as-is
        0,                  // no extra data between each position
        0                   // offset of first element
      );
    }

    if (this->vbo_normals != 0) {
      glEnableVertexAttribArray(attribute_v_normal);
      glBindBuffer(GL_ARRAY_BUFFER, this->vbo_normals);
      glVertexAttribPointer(
        attribute_v_normal, // attribute
        3,                  // number of elements per vertex, here (x,y,z)
        GL_FLOAT,           // the type of each element
        GL_FALSE,           // take our values as-is
        0,                  // no extra data between each position
        0                   // offset of first element
      );
    }

    /* Apply object's transformation matrix */
    glUniformMatrix4fv(uniform_m, 1, GL_FALSE, glm::value_ptr(this->object2world));
    /* Transform normal vectors with transpose of inverse of upper left
       3x3 model matrix (ex-gl_NormalMatrix): */
    glm::mat3 m_3x3_inv_transp = glm::transpose(glm::inverse(glm::mat3(this->object2world)));
    glUniformMatrix3fv(uniform_m_3x3_inv_transp, 1, GL_FALSE, glm::value_ptr(m_3x3_inv_transp));

    /* Push each element in buffer_vertices to the vertex shader */
    if (this->ibo_elements != 0) {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->ibo_elements);
      int size;  glGetBufferParameteriv(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &size);
      glDrawElements(GL_TRIANGLES, size/sizeof(GLushort), GL_UNSIGNED_SHORT, 0);
    } else {
      glDrawArrays(GL_TRIANGLES, 0, this->vertices.size());
    }

    if (this->vbo_normals != 0)
      glDisableVertexAttribArray(attribute_v_normal);
    if (this->vbo_vertices != 0)
      glDisableVertexAttribArray(attribute_v_coord);
  }

/**
* quickly building an edge list for an arbitrary mesh
* http://forum.devmaster.net/t/quickly-building-an-edge-list-for-an-arbitrary-mesh/8605
*/
long Mesh::buildEdges(){

	long vertexCount = vertices.size();
	long triangleCount = triangles.size();
    long maxEdgeCount = triangleCount * 3;
    unsigned short *firstEdge = new unsigned short[vertexCount + maxEdgeCount];
    unsigned short *nextEdge = firstEdge + vertexCount;

    for (long a = 0; a < vertexCount; a++) firstEdge[a] = 0xFFFF;

    // First pass over all triangles. This finds all the edges satisfying the
    // condition that the first vertex index is less than the second vertex index
    // when the direction from the first vertex to the second vertex represents
    // a counterclockwise winding around the triangle to which the edge belongs.
    // For each edge found, the edge index is stored in a linked list of edges
    // belonging to the lower-numbered vertex index <I>i</I>. This allows us to quickly
    // find an edge in the second pass whose higher-numbered vertex index is <I>i</I>.
    long edgeCount = 0;
    for (long a = 0; a < triangleCount; a++)
    {
    	const Triangle triangle = triangles[a];
        long i1 = triangle.index[2];
        for (long b = 0; b < 3; b++)
        {
            long i2 = triangle.index[b];
            if (i1 < i2)
            {
            	Edge edge;
                edge.vertexIndex[0] = (unsigned short) i1;
                edge.vertexIndex[1] = (unsigned short) i2;
                edge.triangleIndex[0] = (unsigned short) a;
                edge.triangleIndex[1] = (unsigned short) a;

                edges.push_back(edge);
                long edgeIndex = firstEdge[i1];
                if (edgeIndex == 0xFFFF)
                {
                    firstEdge[i1] = edgeCount;
                }
                else
                {

                    for (;;)
                    {
                        long index = nextEdge[edgeIndex];
                        if (index == 0xFFFF)
                        {
                           nextEdge[edgeIndex] = edgeCount;
                            break;
                        }

                        edgeIndex = index;
                    }
                }
                nextEdge[edgeCount] = 0xFFFF;
                edgeCount++;
            }

            i1 = i2;
        }
    }

    // Second pass over all triangles. This finds all the edges satisfying the
    // condition that the first vertex index is greater than the second vertex index
    // when the direction from the first vertex to the second vertex represents
    // a counterclockwise winding around the triangle to which the edge belongs.
    // For each of these edges, the same edge should have already been found in
    // the first pass for a different triangle. So we search the list of edges
    // for the higher-numbered vertex index for the matching edge and fill in the
    // second triangle index. The maximum number of comparisons in this search for
    // any vertex is the number of edges having that vertex as an endpoint.
    for (long a = 0; a < triangleCount; a++)
    {
    	const Triangle triangle = triangles[a];
        long i1 = triangle.index[2];
        for (long b = 0; b < 3; b++)
        {
            long i2 = triangle.index[b];
            if (i1 > i2)
            {
                for (long edgeIndex = firstEdge[i2]; edgeIndex != 0xFFFF; edgeIndex = nextEdge[edgeIndex])
                {
                    Edge edge = edges[edgeIndex];
                    if ((edge.vertexIndex[1] == i1) && (edge.triangleIndex[0] == edge.triangleIndex[1]))
                    {
                        edge.triangleIndex[1] = (unsigned short) a;
                        break;
                    }
                }
            }

            i1 = i2;
        }
    }
    delete[] firstEdge;
    return (edgeCount);
}

