//------------------------------------------------------------------------------
// sgng -- Surface-Reconstructing Growing Neural Gas
//
// Developed for my PhD dissertation
//   "Online Surface Reconstruction From Unorganized Point Clouds
//    With Integrated Texture Mapping"
//   Visualization and Computer Graphics Research Group
//   Department of Computer Science
//   University of Muenster
//   Germany
// Copyright (c) 2015, Tom Vierjahn
//------------------------------------------------------------------------------
//                                License
//
// This library/program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// If you are using this library/program in a project, work or publication,
// please cite [1].
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------------------------------------------------
//                                References
//
// [1] Tom Vierjahn, Klaus Hinrichs:
//     "Surface-Reconstructing Growing Neural Gas:
//      A method for online construction of textured triangle meshes".
//     In Computers & Graphics, 51:190–201, 2015.
//     Doi: 10.1016/j.cag.2015.05.016.
//------------------------------------------------------------------------------

#include <gtest/gtest.h>

#include "common/unused.h"
#include "triangle_mesh.h"






////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
TEST(TriangleMeshTest, GetQuadHoleBorderLoop)
{
  TriangleMesh mesh;
  
  // add vertices
  Vertex* pV0 = mesh.NewVertex( -1.0,  -1.0, 0.0);
  Vertex* pV1 = mesh.NewVertex(  1.0,  -1.0, 0.0);
  Vertex* pV2 = mesh.NewVertex(  1.0,   1.0, 0.0);
  Vertex* pV3 = mesh.NewVertex( -1.0,   1.0, 0.0);
  
  Vertex* pV4 = mesh.NewVertex(  0.0, -10.0, 0.0);
  Vertex* pV5 = mesh.NewVertex( 10.0,   0.0, 0.0);
  Vertex* pV6 = mesh.NewVertex(  0.0,  10.0, 0.0);
  Vertex* pV7 = mesh.NewVertex(-10.0,   0.0, 0.0);
  
  // add edges
  Edge* pE01 = mesh.NewEdge(pV0, pV1);
  Edge* pE12 = mesh.NewEdge(pV1, pV2);
  Edge* pE23 = mesh.NewEdge(pV2, pV3);
  Edge* pE30 = mesh.NewEdge(pV3, pV0);
  
  mesh.NewEdge(pV0, pV4);
  mesh.NewEdge(pV1, pV4);
  mesh.NewEdge(pV1, pV5);
  mesh.NewEdge(pV2, pV5);
  mesh.NewEdge(pV2, pV6);
  mesh.NewEdge(pV3, pV6);
  Edge* pE37 = mesh.NewEdge(pV3, pV7);
  mesh.NewEdge(pV0, pV7);
  
  Edge* pE45 = mesh.NewEdge(pV4, pV5);
  mesh.NewEdge(pV5, pV6);
  mesh.NewEdge(pV6, pV7);
  mesh.NewEdge(pV7, pV4);
  
  // add triangles
  mesh.NewTriangle(pV1, pV0, pV4);
  mesh.NewTriangle(pV2, pV1, pV5);
  mesh.NewTriangle(pV3, pV2, pV6);
  mesh.NewTriangle(pV0, pV3, pV7);
  
  mesh.NewTriangle(pV4, pV0, pV7);
  mesh.NewTriangle(pV5, pV1, pV4);
  mesh.NewTriangle(pV6, pV2, pV5);
  mesh.NewTriangle(pV7, pV3, pV6);
  
  Edge* pLoopEdges[4] = { NULL, NULL, NULL, NULL };
  Vertex* pLoopVertices[4] = { NULL, NULL, NULL, NULL };
  bool foundLoop = mesh.GetQuadHoleBorderLoop(pE01, pLoopEdges, pLoopVertices);
  
  EXPECT_TRUE(foundLoop);
  
  EXPECT_EQ(pE01, pLoopEdges[0]);
  EXPECT_EQ(pE12, pLoopEdges[1]);
  EXPECT_EQ(pE23, pLoopEdges[2]);
  EXPECT_EQ(pE30, pLoopEdges[3]);
  
  EXPECT_EQ(pV0, pLoopVertices[0]);
  EXPECT_EQ(pV1, pLoopVertices[1]);
  EXPECT_EQ(pV2, pLoopVertices[2]);
  EXPECT_EQ(pV3, pLoopVertices[3]);
  
  
  pLoopEdges[0] = NULL;
  pLoopEdges[1] = NULL;
  pLoopEdges[2] = NULL;
  pLoopEdges[3] = NULL;
  
  pLoopVertices[0] = NULL;
  pLoopVertices[1] = NULL;
  pLoopVertices[2] = NULL;
  pLoopVertices[3] = NULL;
  
  foundLoop = mesh.GetQuadHoleBorderLoop(pE45, pLoopEdges, pLoopVertices);
  
  EXPECT_FALSE(foundLoop);
  
  EXPECT_EQ(NULL, pLoopEdges[0]);
  EXPECT_EQ(NULL, pLoopEdges[1]);
  EXPECT_EQ(NULL, pLoopEdges[2]);
  EXPECT_EQ(NULL, pLoopEdges[3]);
  
  EXPECT_EQ(NULL, pLoopVertices[0]);
  EXPECT_EQ(NULL, pLoopVertices[1]);
  EXPECT_EQ(NULL, pLoopVertices[2]);
  EXPECT_EQ(NULL, pLoopVertices[3]);
  
  
  pLoopEdges[0] = NULL;
  pLoopEdges[1] = NULL;
  pLoopEdges[2] = NULL;
  pLoopEdges[3] = NULL;
  
  pLoopVertices[0] = NULL;
  pLoopVertices[1] = NULL;
  pLoopVertices[2] = NULL;
  pLoopVertices[3] = NULL;
  
  foundLoop = mesh.GetQuadHoleBorderLoop(pE37, pLoopEdges, pLoopVertices);
  
  EXPECT_FALSE(foundLoop);
  
  EXPECT_EQ(NULL, pLoopEdges[0]);
  EXPECT_EQ(NULL, pLoopEdges[1]);
  EXPECT_EQ(NULL, pLoopEdges[2]);
  EXPECT_EQ(NULL, pLoopEdges[3]);
  
  EXPECT_EQ(NULL, pLoopVertices[0]);
  EXPECT_EQ(NULL, pLoopVertices[1]);
  EXPECT_EQ(NULL, pLoopVertices[2]);
  EXPECT_EQ(NULL, pLoopVertices[3]);
}


//////////////////////////////////////////////////////////////////////////////////
///// 
//////////////////////////////////////////////////////////////////////////////////
//class TriangleMeshTest : public ::testing::Test
//{
//public:
//  TriangleMeshTest() 
//  {
//    // add vertices
//    this->fpV0 = this->fTriangleMesh.NewVertex(-1.0, -1.0, 0.0);
//    this->fpV1 = this->fTriangleMesh.NewVertex( 1.0, -1.0, 0.0);
//    this->fpV2 = this->fTriangleMesh.NewVertex( 1.0,  1.0, 0.0);
//    this->fpV3 = this->fTriangleMesh.NewVertex(-1.0,  1.0, 0.0);
//    
//    // add triangles
//    this->fpTri0 = this->fTriangleMesh.NewTriangle(this->fpV0, 
//                                                   this->fpV1, 
//                                                   this->fpV2);
//    this->fpTri1 = this->fTriangleMesh.NewTriangle(this->fpV2, 
//                                                   this->fpV3, 
//                                                   this->fpV0);
//  }
//  
//protected:
//  TriangleMesh fTriangleMesh;
//  
//  Vertex* fpV0;
//  Vertex* fpV1;
//  Vertex* fpV2;
//  Vertex* fpV3;
//  
//  Triangle* fpTri0;
//  Triangle* fpTri1;
//};
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////
///// 
//////////////////////////////////////////////////////////////////////////////////
//TEST_F(TriangleMeshTest, DISABLE_GetNumOfTriangles)
//{
//  EXPECT_EQ(2u, this->fTriangleMesh.GetNumOfTriangles());
//  
//  this->fTriangleMesh.NewTriangle(this->fpV0, this->fpV2, this->fpV3);
//  EXPECT_EQ(3u, this->fTriangleMesh.GetNumOfTriangles());
//}
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////
///// 
//////////////////////////////////////////////////////////////////////////////////
//TEST_F(TriangleMeshTest, DISABLE_GetNumOfVertices)
//{
//  EXPECT_EQ(4u, this->fTriangleMesh.GetNumOfVertices());
//  
//  this->fTriangleMesh.NewVertex(0.0, 0.0, 0.0);
//  EXPECT_EQ(5u, this->fTriangleMesh.GetNumOfVertices());
//}
