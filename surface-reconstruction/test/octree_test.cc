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

#include <cstdlib>

#include <common/unused.h>

#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#endif

#include <gtest/gtest.h>

#include "data_types.h"
#include "octree.h"
#include "triangle_mesh.h"


template <typename T>
Flt random(T min = sc<T>(0.0), T max = sc<T>(1.0))
{
  const Flt x = sc<Flt>(rand()) / sc<Flt>(RAND_MAX);
  return (sc<Flt>(max) - sc<Flt>(min)) * x + sc<Flt>(min);
}

template <typename T>
Position randomPosition(T min = sc<T>(0.0), T max = sc<T>(1.0))
{
  return Position(random(min, max), random(min, max), random(min, max)); 
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, InsertAndFindEightVertices)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>(1.0)));
  
  const Position position[8] = 
    { randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0), 
      randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0), 
      randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0),
      randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0) };
  
  Vertex* pNewVertex[8] = 
    { mesh.NewVertex(position[0]), mesh.NewVertex(position[1]), 
      mesh.NewVertex(position[2]), mesh.NewVertex(position[3]),
      mesh.NewVertex(position[4]), mesh.NewVertex(position[5]),
      mesh.NewVertex(position[6]), mesh.NewVertex(position[7]) };
  
  octree.Insert(pNewVertex[0]);
  octree.Insert(pNewVertex[1]);
  octree.Insert(pNewVertex[2]);
  octree.Insert(pNewVertex[3]);
  octree.Insert(pNewVertex[4]);
  octree.Insert(pNewVertex[5]);
  octree.Insert(pNewVertex[6]);
  octree.Insert(pNewVertex[7]);
  
  Vertex* pFoundVertex[8] =
    { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
  
  pFoundVertex[0] = octree.FindNearestNeighbour(position[0]);
  pFoundVertex[1] = octree.FindNearestNeighbour(position[1]);
  pFoundVertex[2] = octree.FindNearestNeighbour(position[2]);
  pFoundVertex[3] = octree.FindNearestNeighbour(position[3]);
  pFoundVertex[4] = octree.FindNearestNeighbour(position[4]);
  pFoundVertex[5] = octree.FindNearestNeighbour(position[5]);
  pFoundVertex[6] = octree.FindNearestNeighbour(position[6]);
  pFoundVertex[7] = octree.FindNearestNeighbour (position[7]);
  
  EXPECT_EQ(pNewVertex[0], pFoundVertex[0]);
  EXPECT_EQ(pNewVertex[1], pFoundVertex[1]);
  EXPECT_EQ(pNewVertex[2], pFoundVertex[2]);
  EXPECT_EQ(pNewVertex[3], pFoundVertex[3]);
  EXPECT_EQ(pNewVertex[4], pFoundVertex[4]);
  EXPECT_EQ(pNewVertex[5], pFoundVertex[5]);
  EXPECT_EQ(pNewVertex[6], pFoundVertex[6]);
  EXPECT_EQ(pNewVertex[7], pFoundVertex[7]);
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, InsertNineVertices)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>(1.0)));
  
  const Position position[9] = 
  { randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0), 
    randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0), 
    randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0),
    randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0),
    randomPosition(-1.0, 1.0) };
  
  Vertex* pNewVertex[9] = 
  { mesh.NewVertex(position[0]), mesh.NewVertex(position[1]), 
    mesh.NewVertex(position[2]), mesh.NewVertex(position[3]),
    mesh.NewVertex(position[4]), mesh.NewVertex(position[5]),
    mesh.NewVertex(position[6]), mesh.NewVertex(position[7]),
    mesh.NewVertex(position[8]) };
  
  octree.Insert(pNewVertex[0]);
  octree.Insert(pNewVertex[1]);
  octree.Insert(pNewVertex[2]);
  octree.Insert(pNewVertex[3]);
  octree.Insert(pNewVertex[4]);
  octree.Insert(pNewVertex[5]);
  octree.Insert(pNewVertex[6]);
  octree.Insert(pNewVertex[7]);
  octree.Insert(pNewVertex[8]);
  
  OctreeNode* pRootNode = octree.GetRoot();
  ASSERT_TRUE(pRootNode != NULL);
  
  EXPECT_FALSE(pRootNode->IsLeaf());
  
  EXPECT_TRUE(pRootNode->GetChild(0)->IsLeaf());
  EXPECT_TRUE(pRootNode->GetChild(1)->IsLeaf());
  EXPECT_TRUE(pRootNode->GetChild(2)->IsLeaf());
  EXPECT_TRUE(pRootNode->GetChild(3)->IsLeaf());
  EXPECT_TRUE(pRootNode->GetChild(4)->IsLeaf());
  EXPECT_TRUE(pRootNode->GetChild(5)->IsLeaf());
  EXPECT_TRUE(pRootNode->GetChild(6)->IsLeaf());
  EXPECT_TRUE(pRootNode->GetChild(7)->IsLeaf());
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, InsertTwoGenerations)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>(1.0)));
  
  Flt borders[] = 
    { sc<Flt>(-1.00), sc<Flt>(-0.75), sc<Flt>(-0.50),
      sc<Flt>(-0.25), sc<Flt>( 0.00), sc<Flt>( 0.25),
      sc<Flt>( 0.50), sc<Flt>( 0.75), sc<Flt>( 1.00) };
  
  for (unsigned int x = 0; x < 8; ++x)
  {
    for (unsigned int y = 0; y < 8; ++y)
    {
      for (unsigned int z = 0; z < 8; ++z)
      {
        octree.Insert(mesh.NewVertex(random(borders[x], borders[x+1]),
                                     random(borders[y], borders[y+1]),
                                     random(borders[z], borders[z+1])));
      }
    }
  }
  
  
  OctreeNode* pRootNode = octree.GetRoot();
  for (unsigned int i = 0; i < 8; ++i)
  {
    EXPECT_FALSE(pRootNode->GetChild(i)->IsLeaf());
    for (unsigned int j = 0; j < 8; ++j)
    {
      EXPECT_TRUE(pRootNode->GetChild(i)->GetChild(j)->IsLeaf());
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, InsertTwoGenerationsAsymmetric)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  Flt borders[] = 
  { -1.00, -0.75, -0.50, -0.25, 0.00,
    0.25,  0.50,  0.75,  1.00 };
  
  for (unsigned int x = 0; x < 8; ++x)
  {
    for (unsigned int y = 0; y < 8; ++y)
    {
      for (unsigned int z = 0; z < 1; ++z)
      {
        octree.Insert(mesh.NewVertex(random(borders[x], borders[x+1]),
                                     random(borders[y], borders[y+1]),
                                     random(borders[z], borders[z+1])));
      }
    }
  }
  
  
  OctreeNode* pRootNode = octree.GetRoot();
  for (unsigned int i = 0; i < 4; ++i)
  {
    EXPECT_FALSE(pRootNode->GetChild(i)->IsLeaf());
    for (unsigned int j = 0; j < 4; ++j)
    {
      EXPECT_TRUE(pRootNode->GetChild(i)->GetChild(j)->IsLeaf());
    }
  }
  for (unsigned int i = 5; i < 8; ++i)
  {
    EXPECT_TRUE(pRootNode->GetChild(i)->IsLeaf());
  }
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, DISABLED_Insert1MPoints)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  for (unsigned int i = 0; i < 1000000; ++i)
  {
    EXPECT_NO_FATAL_FAILURE(
      octree.Insert(mesh.NewVertex(random(-1.0, 1.0),
                                   random(-1.0, 1.0),
                                   random(-1.0, 1.0))));
  }
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, InsertPointsOutsideRootNode)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  // insert 16 points into octree, i.e., make root node completely full
  for (unsigned int i = 0; i < 16; ++i)
  {
    octree.Insert(mesh.NewVertex(randomPosition(-1.0, 1.0)));
  }
  
  // insert a point outside the original root node
  octree.Insert(mesh.NewVertex(randomPosition(-3.0, -1.0)));
  
  OctreeNode* pRootNode = octree.GetRoot();
  ASSERT_TRUE(pRootNode != NULL);
  EXPECT_FALSE(pRootNode->IsLeaf());

  ASSERT_TRUE(pRootNode->GetChild(0) != NULL);
  EXPECT_TRUE(pRootNode->GetChild(0)->IsLeaf());
  ASSERT_TRUE(pRootNode->GetChild(1) != NULL);
  EXPECT_TRUE(pRootNode->GetChild(1)->IsLeaf());
  ASSERT_TRUE(pRootNode->GetChild(2) != NULL);
  EXPECT_TRUE(pRootNode->GetChild(2)->IsLeaf());
  ASSERT_TRUE(pRootNode->GetChild(3) != NULL);
  EXPECT_TRUE(pRootNode->GetChild(3)->IsLeaf());
  ASSERT_TRUE(pRootNode->GetChild(4) != NULL);
  EXPECT_TRUE(pRootNode->GetChild(4)->IsLeaf());
  ASSERT_TRUE(pRootNode->GetChild(5) != NULL);
  EXPECT_TRUE(pRootNode->GetChild(5)->IsLeaf());
  ASSERT_TRUE(pRootNode->GetChild(6) != NULL);
  EXPECT_TRUE(pRootNode->GetChild(6)->IsLeaf());
  
  ASSERT_TRUE(pRootNode->GetChild(7) != NULL);
  EXPECT_FALSE(pRootNode->GetChild(7)->IsLeaf());
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, InsertAndRemoveEightVertices)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  const Position position[8] = 
  { randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0), 
    randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0), 
    randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0),
    randomPosition(-1.0, 1.0), randomPosition(-1.0, 1.0) };
  
  Vertex* pNewVertex[8] = 
  { mesh.NewVertex(position[0]), mesh.NewVertex(position[1]), 
    mesh.NewVertex(position[2]), mesh.NewVertex(position[3]),
    mesh.NewVertex(position[4]), mesh.NewVertex(position[5]),
    mesh.NewVertex(position[6]), mesh.NewVertex(position[7]) };
  
  octree.Insert(pNewVertex[0]);
  octree.Insert(pNewVertex[1]);
  octree.Insert(pNewVertex[2]);
  octree.Insert(pNewVertex[3]);
  octree.Insert(pNewVertex[4]);
  octree.Insert(pNewVertex[5]);
  octree.Insert(pNewVertex[6]);
  octree.Insert(pNewVertex[7]);
  
  // remove all except one
  EXPECT_TRUE(octree.RemoveElement(pNewVertex[0]));
  EXPECT_TRUE(octree.RemoveElement(pNewVertex[1]));
  EXPECT_TRUE(octree.RemoveElement(pNewVertex[2]));
  EXPECT_TRUE(octree.RemoveElement(pNewVertex[3]));
  EXPECT_TRUE(octree.RemoveElement(pNewVertex[4]));
  EXPECT_TRUE(octree.RemoveElement(pNewVertex[5]));
  EXPECT_TRUE(octree.RemoveElement(pNewVertex[6]));
  
  OctreeNode* pRootNode = octree.GetRoot();
  ASSERT_TRUE(pRootNode != NULL);
  EXPECT_TRUE(pRootNode->IsLeaf());
  EXPECT_EQ(1, pRootNode->NumOfLeafElements());
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, InsertNineRemoveOneExpectCollapse)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  
  for (unsigned int i = 0; i < 8; ++i)
  {
    octree.Insert(mesh.NewVertex(randomPosition(-1.0, 1.0)));
  }
  Vertex* pLastVertex  = mesh.NewVertex(randomPosition(-1.0, 1.0));
  octree.Insert(pLastVertex);
  
  OctreeNode* pRootNode = octree.GetRoot();
  ASSERT_TRUE(pRootNode != NULL);
  ASSERT_FALSE(pRootNode->IsLeaf());
  
  octree.RemoveElement(pLastVertex);
  
  EXPECT_TRUE(pRootNode->IsLeaf());
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, DISABLED_Insert10kPointsPerformance)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    EXPECT_NO_FATAL_FAILURE(octree.Insert(mesh.NewVertex(random(-1.0, 1.0),
                                                         random(-1.0, 1.0),
                                                         random(-1.0, 1.0))));
  }
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, DISABLED_Insert10kPointsAndFind10kOctreePerformance)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    EXPECT_NO_FATAL_FAILURE(octree.Insert(mesh.NewVertex(random(-1.0, 1.0),
                                                         random(-1.0, 1.0),
                                                         random(-1.0, 1.0))));
  }
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    const Position randomPos(random(-1.0, 1.0),
                                     random(-1.0, 1.0),
                                     random(-1.0, 1.0));
    Vertex* pFoundByOctree = octree.FindNearestNeighbour(randomPos);
    EXPECT_TRUE(pFoundByOctree != NULL);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, DISABLED_Insert10kPointsAndFind10kLinearPerformance)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    EXPECT_NO_FATAL_FAILURE(octree.Insert(mesh.NewVertex(random(-1.0, 1.0),
                                                         random(-1.0, 1.0),
                                                         random(-1.0, 1.0))));
  }
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    const Position randomPos(random(-1.0, 1.0),
                                     random(-1.0, 1.0),
                                     random(-1.0, 1.0));
    Vertex* pFoundByLinear = NULL;
    Flt minSquaredDistance =
    std::numeric_limits<Flt>::infinity();
    BOOST_FOREACH(Vertex& vtx, mesh.GetVertices())
    {
      Flt currentSquaredDistance =
        (vtx.GetPosition() - randomPos).GetSquaredLength();
      if (currentSquaredDistance < minSquaredDistance)
      {
        minSquaredDistance = currentSquaredDistance;
        pFoundByLinear = &vtx;
      }
    }
    
    EXPECT_TRUE(pFoundByLinear != NULL);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, DISABLED_Insert10kPointsAndFind10k)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    octree.Insert(mesh.NewVertex(random(-1.0, 1.0),
                                 random(-1.0, 1.0),
                                 random(-1.0, 1.0)));
  }
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    const Position randomPos(random(-1.0, 1.0),
                                     random(-1.0, 1.0),
                                     random(-1.0, 1.0));
    Vertex* pFoundByOctree = octree.FindNearestNeighbour(randomPos);
    
    Vertex* pFoundByLinear = NULL;
    Flt minSquaredDistance =
      std::numeric_limits<Flt>::infinity();
    BOOST_FOREACH(Vertex& vtx, mesh.GetVertices())
    {
      Flt currentSquaredDistance =
        (vtx.GetPosition() - randomPos).GetSquaredLength();
      if (currentSquaredDistance < minSquaredDistance)
      {
        minSquaredDistance = currentSquaredDistance;
        pFoundByLinear = &vtx;
      }
    }
    
    EXPECT_EQ(pFoundByLinear, pFoundByOctree);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, DISABLED_Insert10kPointsAndFindTwo10kPerformance)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    EXPECT_NO_FATAL_FAILURE(octree.Insert(mesh.NewVertex(random(-1.0, 1.0),
                                                         random(-1.0, 1.0),
                                                         random(-1.0, 1.0))));
  }
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    const Position randomPos(random(-1.0, 1.0),
                                     random(-1.0, 1.0),
                                     random(-1.0, 1.0));
    Vertex* pFoundByOctree0 = NULL;
    Vertex* pFoundByOctree1 = NULL;
    
    Flt minSquaredDistanceByOctree0 =
    std::numeric_limits<Flt>::infinity();
    Flt minSquaredDistanceByOctree1 =
    std::numeric_limits<Flt>::infinity();
    
    octree.FindTwoNearestNeighbour(randomPos,
                                   &pFoundByOctree0,
                                   &minSquaredDistanceByOctree0,
                                   &pFoundByOctree1,
                                   &minSquaredDistanceByOctree1);
    
    EXPECT_TRUE(pFoundByOctree0 != NULL);
    EXPECT_TRUE(pFoundByOctree1 != NULL);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TEST(OctreeTest, DISABLED_Insert10kPointsAndFindTwo10k)
{
  TriangleMesh mesh;
  Octree octree(Position(sc<Flt>(-1.0)),
                Position(sc<Flt>( 1.0)));
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    octree.Insert(mesh.NewVertex(random(-1.0, 1.0),
                                 random(-1.0, 1.0),
                                 random(-1.0, 1.0)));
  }
  
  for (unsigned int i = 0; i < 10000; ++i)
  {
    const Position randomPos(random(-1.0, 1.0),
                                     random(-1.0, 1.0),
                                     random(-1.0, 1.0));
    Vertex* pFoundByOctree0 = NULL;
    Vertex* pFoundByOctree1 = NULL;
    
    Flt minSquaredDistanceByOctree0 =
      std::numeric_limits<Flt>::infinity();
    Flt minSquaredDistanceByOctree1 =
      std::numeric_limits<Flt>::infinity();
    
    octree.FindTwoNearestNeighbour(randomPos,
                                   &pFoundByOctree0,
                                   &minSquaredDistanceByOctree0,
                                   &pFoundByOctree1,
                                   &minSquaredDistanceByOctree1);
    
    Vertex* pFoundByLinear0 = NULL;
    Vertex* pFoundByLinear1 = NULL;
    Flt minSquaredDistanceByLinear0 =
      std::numeric_limits<Flt>::infinity();
    Flt minSquaredDistanceByLinear1 =
      std::numeric_limits<Flt>::infinity();
    BOOST_FOREACH(Vertex& vtx, mesh.GetVertices())
    {
      Flt currentSquaredDistance =
        (vtx.GetPosition() - randomPos).GetSquaredLength();
      
      if (currentSquaredDistance < minSquaredDistanceByLinear0)
      {
        minSquaredDistanceByLinear1 = minSquaredDistanceByLinear0;
        minSquaredDistanceByLinear0 = currentSquaredDistance;
        
        pFoundByLinear1 = pFoundByLinear0;
        pFoundByLinear0 = &vtx;
      }
      else if (currentSquaredDistance < minSquaredDistanceByLinear1)
      {
        minSquaredDistanceByLinear1 = currentSquaredDistance;
        
        pFoundByLinear1 = &vtx;
      }
    }
    
    EXPECT_EQ(pFoundByLinear0, pFoundByOctree0);
    EXPECT_EQ(pFoundByLinear1, pFoundByOctree1);
  }
}
