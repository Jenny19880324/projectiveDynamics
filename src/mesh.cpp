#include "mesh.h"

bool ClothMesh::Init()
{
    generateParticleList();
    generateTriangleList();
    generateEdgeList();
}
