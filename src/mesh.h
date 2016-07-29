#ifndef _MESH_H_
#define _MESH_H_

struct Edge
{
    unsigned int m_v[2]; // indices of endpoint vertices
    unsigned_int m_f[2]; // indices of adjacent faces
}

class Mesh
{
public:
    Mesh(){}
    ~Mesh(){}

    void Draw();
    void Init();

private:
    unsigned int m_vertices_number;

    std::vector<Edge> m_edge_list;
    std::vector<unsigned int> m_triangle_list;
    std::vector<Eigen::Vector3f> m_positions;
    std::vector<Eigen::Vector3f> m_normals;

    unsigned int width;
    unsigned int length;
    Eigen::Vector3f m_corners[2];

    void generateParticleList();
    void generateTriangleList();
    void generateEdgeList();
}
