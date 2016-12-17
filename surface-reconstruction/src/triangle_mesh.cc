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

#include <algorithm>
#include <iostream>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <no_warning/boost__lambda__lambda.h>
#include <no_warning/boost__lambda__bind.h>
#endif

#include <common/logging.h>

#include "adjacency.h"
#include "container.h"
#include "data_types.h"
#include "octree.h"
#include "program_options.h"
#include "triangle_mesh.h"





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TriangleMesh::TriangleMesh
()
: fTotalActivity(sc<Activity>(0))
, fTotalBmuCount(0u)
, fpOctree(new Octree(Position(-10), Position(10)))
, fDirty(true)
, fPointsPerVertex(ProgramOptions::GetPointsPerVertex())
{
  this->AllocateTriangles(5000);
  
  this->AllocateEdges(10000);
  
  this->AllocateVertices(5000);
  
  //this->fInactiveVerticesBegin = this->fVertices.end();
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
TriangleMesh::~TriangleMesh
()
{
  this->fUnusedTriangles.clear_and_dispose(boost::bind(&Triangle::Delete, _1));
  this->fTriangles.clear_and_dispose(boost::bind(&Triangle::Delete, _1));
  
  this->fUnusedEdges.clear_and_dispose(boost::bind(&Edge::Delete, _1));
  this->fUnusedEdges.clear_and_dispose(boost::bind(&Edge::Delete, _1));
  
  this->fInactiveVertices.clear();
  this->fVertices.clear_and_dispose(boost::bind(&Vertex::Delete, _1));
  this->fUnusedVertices.clear_and_dispose(boost::bind(&Vertex::Delete, _1));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::ResetVerticesEdgesTriangles
()
{
  
  std::vector<Triangle*> triangles;
  BOOST_FOREACH(Triangle& tri, this->GetTriangles())
  {
    triangles.push_back(&tri);
  }
  BOOST_FOREACH(Triangle* pTri, triangles)
  {
    this->DeleteTriangle(pTri);
  }
  
  std::vector<Edge*> edges;
  BOOST_FOREACH(Edge& edge, this->GetEdges())
  {
    edges.push_back(&edge);
  }
                
  BOOST_FOREACH(Edge* pEdge, edges)
  {
    this->DeleteEdge(pEdge);
  }
  
  std::vector<Vertex*> vertices;
  BOOST_FOREACH(Vertex& vert, this->GetVertices())
  {
    vertices.push_back(&vert);
  }
  BOOST_FOREACH(Vertex* pVert, vertices)
  {
    this->DeleteVertex(pVert);
  }
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void TriangleMesh::Save(const io::DataStorage<TriangleMesh>& dataStorage)
{
  dataStorage.Save(this);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
Triangle*
TriangleMesh::NewTriangle
(Vertex *pV0, Vertex *pV1, Vertex *pV2)
{
  if ((pV0 == pV1) || (pV0 == pV2) || (pV1 == pV2))
  {
    return NULL;
  }
  
  Triangle* pTri = NULL;
  
  assert(pV0 != NULL);
  assert(pV1 != NULL);
  assert(pV2 != NULL);
  
  // Find exisiting edges between V0, V1 and V2
  Edge* pEdge01 = this->GetEdge(pV0, pV1);
  Edge* pEdge12 = this->GetEdge(pV1, pV2);
  Edge* pEdge20 = this->GetEdge(pV2, pV0);
  
  // all edges must exist
  assert(pEdge01 != NULL);
  assert(pEdge12 != NULL);
  assert(pEdge20 != NULL);
  
  // add triangles only to edges of valence 0 or valence 1
  if ((pEdge01->GetValence() > 1) || 
      (pEdge12->GetValence() > 1) ||
      (pEdge20->GetValence() > 1))
  {
    return NULL;
  }
  
  // create triangle
  if (this->fUnusedTriangles.size() == 1)
  {
    this->AllocateTriangles(5000);
  }
  
  this->fTriangles.splice(this->fTriangles.end(),
                          this->fUnusedTriangles,
                          this->fUnusedTriangles.begin());
  pTri = &(this->fTriangles.back());
  
  pTri->Init();
  
  pTri->SetEdges(pEdge01, pEdge12, pEdge20);

  const Vector edgeA(pV1->GetPosition() - pV0->GetPosition());
  const Vector edgeB(pV2->GetPosition() - pV0->GetPosition());
  const Vector triNormal(crossProduct(edgeB, edgeB));
  pTri->SetNormal(triNormal);
  
  this->fDirty = true;
  
  return pTri;
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
Edge*
TriangleMesh::NewEdge
(Vertex* pV0, Vertex* pV1)
{
  Edge* pEdge = NULL;
  if (this->fUnusedEdges.size() == 1)
  {
    this->AllocateEdges(10000);
  }
  
  this->fEdges.splice(this->fEdges.end(),
                      this->fUnusedEdges,
                      this->fUnusedEdges.begin());
  pEdge = &(this->fEdges.back());
  
  pEdge->Init();
  
  pEdge->SetVertices(pV0, pV1);

  this->fDirty = true;
  
  return pEdge;
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
Vertex*
TriangleMesh::NewVertex
()
{
  // allocate new vertices if necessary
  if (this->fUnusedVertices.size() == 1)
  {
    this->AllocateVertices(5000);
  }
  
  Vertex* pNewVertex = &(this->fUnusedVertices.front());
  const Activity newActivity =
    (this->fVertices.size() > sc<Activity>(0u)) ?
    this->fVertices.back().GetActivity() :
    sc<Activity>(0);
  
  // insert new vertex at the end of the active vertices
  this->fVertices.splice(this->fVertices.end(),
                         this->fUnusedVertices,
                         this->fUnusedVertices.begin());
  // insert new vertex at the front of the inactive list
  this->fInactiveVertices.insert(this->fInactiveVertices.begin(),
                                 *pNewVertex);
  
  // init
  pNewVertex->Init();

  pNewVertex->SetActivity(newActivity);
  this->fTotalActivity += pNewVertex->GetActivity();

  this->fDirty = true;
  
  return pNewVertex;
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::AllocateTriangles
(unsigned int num)
{
  // allocate heaps of triangles
  for(unsigned int i = 0; i < num; ++i)
  {
    this->fUnusedTriangles.push_back(*(new Triangle));
  }
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::AllocateEdges
(unsigned int num)
{
  this->fUnusedEdges.clear_and_dispose(boost::bind(&Edge::Delete, _1));
  // allocate heaps of edges
  for(unsigned int i = 0; i < num; ++i)
  {
    this->fUnusedEdges.push_back(*(new Edge));
  }
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::AllocateVertices
(unsigned int num)
{
  // allocate heaps of vertices
  for(unsigned int i = 0; i < num; ++i)
  {
    this->fUnusedVertices.push_back(*(new Vertex));
  }
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::DeleteTriangle
(Triangle *pTriangle,
 std::vector<Edge*>* pEdgesToRepair)
{
  if (pTriangle != NULL)
  {
    // store edges of which the valence gets reduced by this operation
    if (pEdgesToRepair != NULL)
    {
      pEdgesToRepair->push_back(pTriangle->GetE0());
      pEdgesToRepair->push_back(pTriangle->GetE1());
      pEdgesToRepair->push_back(pTriangle->GetE2());
    }
    
    pTriangle->SetEdges(NULL, NULL, NULL);
    
    this->fUnusedTriangles.splice(this->fUnusedTriangles.end(),
                                  this->fTriangles,
                                  this->fTriangles.iterator_to(*pTriangle));
  }
  this->fDirty = true;
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::DeleteEdge
(Edge* pEdge,
 bool deleteDanglingVertices,
 std::vector<Edge*>* pEdgesToRepair)
{
  if (pEdge != NULL)
  {
    Triangle* const pAdjTri0 = pEdge->GetT0();
    Triangle* const pAdjTri1 = pEdge->GetT1();
    
    // delete adjacent Triangles
    if (pAdjTri0 != NULL)
    {
      this->DeleteTriangle(pAdjTri0, pEdgesToRepair);
    }
    if (pAdjTri1 != NULL)
    {
      this->DeleteTriangle(pAdjTri1, pEdgesToRepair);
    }
    
    Vertex* const pV0 = pEdge->GetV0();
    Vertex* const pV1 = pEdge->GetV1();
    
    pEdge->SetVertices(NULL, NULL);
    pEdge->SetIsActive(false);
    
    this->fUnusedEdges.splice(this->fUnusedEdges.end(),
                              this->fEdges,
                              this->fEdges.iterator_to(*pEdge));
  
    if (deleteDanglingVertices)
    {
      if ((pV0 != NULL) && pV0->IsDangling())
      {
        this->DeleteVertex(pV0);
      }
      if ((pV1 != NULL) && pV1->IsDangling())
      {
        this->DeleteVertex(pV1);
      }
    }
  }
  this->fDirty = true;
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::DeleteVertex
(Vertex *pVertex)
{
  if (pVertex != NULL)
  {
    bool success = this->fpOctree->RemoveElement(pVertex);
    assert(success);
    UNUSED(success);
    
    this->fTotalActivity -= pVertex->GetActivity();
    this->fInactiveVertices.erase(
      this->fInactiveVertices.iterator_to(*pVertex));
        
    this->fUnusedVertices.splice(this->fUnusedVertices.end(),
                                 this->fVertices,
                                 this->fVertices.iterator_to(*pVertex));
  }
  this->fDirty = true;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::DeleteAllTriangles
()
{
  this->fUnusedTriangles.splice(this->fUnusedTriangles.end(),
                                this->fTriangles,
                                this->fTriangles.begin(),
                                this->fTriangles.end());
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::DeleteAllEdges
()
{
  this->fUnusedEdges.splice(this->fUnusedEdges.end(),
                            this->fEdges,
                            this->fEdges.begin(),
                            this->fEdges.end());
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::DeleteAllVertices
()
{
  this->fUnusedVertices.splice(this->fUnusedVertices.end(),
                               this->fVertices,
                               this->fVertices.begin(),
                               this->fVertices.end());
  this->fInactiveVertices.clear();
  this->fTotalActivity = sc<Activity>(0);
  this->fTotalBmuCount = 0u;
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::FindClosestVertices
(const Position &position, 
 Vertex* closestVertices[2])
{
  Flt closestDistance[2] = { 
    std::numeric_limits<Flt>::infinity(),
    std::numeric_limits<Flt>::infinity() };
  
  this->fpOctree->FindTwoNearestNeighbour(position,
                                          &closestVertices[0], 
                                          &closestDistance[0], 
                                          &closestVertices[1], 
                                          &closestDistance[1]);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::FindCommonNeighbours
(Vertex* pVA,
 Vertex* pVB,
 std::vector<Vertex*>* pCommonNeighbours)
{
  if (pVA == pVB)
    return;
  assert(pVA != pVB);
  assert(pCommonNeighbours->size() == 0);
  pCommonNeighbours->clear();
  
  Vertex* pVtx1 = (pVA->GetValence() <= pVB->GetValence()) ? pVA : pVB;
  Vertex* pVtx2 = (pVA->GetValence() <= pVB->GetValence()) ? pVB : pVA;
  
  // store neighbours of pVtx1
  typedef std::vector<Vertex*> ContainerType;
  ContainerType neighboursOf1;
  neighboursOf1.reserve(pVtx1->GetValence());
  
  BOOST_FOREACH(Edge* pEdge, Adjacent<Edge>::Around(pVtx1))
  {
    assert(pEdge->GetIsActive());
    neighboursOf1.push_back(pEdge->GetOther(pVtx1));
  }
  
  // check every neighbour of pVtx2 whether it is in neighboursOf1
  BOOST_FOREACH(Edge* pEdge, Adjacent<Edge>::Around(pVtx2))
  {
    assert(pEdge->GetIsActive());
    Vertex* const pVtxOther = pEdge->GetOther(pVtx2);
    ContainerType::const_iterator found(std::find(neighboursOf1.begin(),
                                                  neighboursOf1.end(),
                                                  pVtxOther));
    if (found != neighboursOf1.end())
    {
      pCommonNeighbours->push_back(pVtxOther);
    }
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::SelectTwoLeastRoughNeighbours
(Vertex* pVtxA,
 Vertex* pVtxB,
 std::vector<Vertex*>* pCommonNeighbours)
{
  Flt lowestRoughness =
    std::numeric_limits<Flt>::infinity();
  Vertex* pVtx0 = NULL;
  Vertex* pVtx1 = NULL;
  
  typedef std::vector<Vertex*> VtxContainer;
  VtxContainer::iterator neighbourIter0 = pCommonNeighbours->begin();
  VtxContainer::iterator neighbourIter1 = pCommonNeighbours->begin();
  VtxContainer::const_iterator neighboursEnd = pCommonNeighbours->end();
  for (; neighbourIter0 != neighboursEnd; ++neighbourIter0)
  {
    neighbourIter1 = neighbourIter0;
    ++neighbourIter1;
    for (; neighbourIter1 != neighboursEnd; ++neighbourIter1)
    {
      if ((*neighbourIter0) == (*neighbourIter1))
      {
        continue;
      }
      const Flt roughness =
      this->GetRoughness(pVtxA,
                         pVtxB,
                         *neighbourIter0,
                         *neighbourIter1);
      
      if ((roughness < lowestRoughness))
      {
        lowestRoughness = roughness;
        pVtx0 = *neighbourIter0;
        pVtx1 = *neighbourIter1;
      }
    }
  }
  pCommonNeighbours->clear();
  if (pVtx0 != NULL)
  {
    pCommonNeighbours->push_back(pVtx0);
  }
  if (pVtx1 != NULL)
  {
    pCommonNeighbours->push_back(pVtx1);
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::IncrementVertexActivity
(Vertex *pVertex,
 unsigned int numOfPoints)
{
  // increment activity
  pVertex->IncrementActivity();
  this->fTotalActivity += Vertex::ActivityIncrement();
  
  // sort fVertices
  typedef ::Container<Vertex> CV;
  const CV::Iterator vtxIter(this->fVertices.iterator_to(*pVertex));
  if (vtxIter != this->fVertices.begin())
  {
    CV::Iterator predIter(vtxIter);
    --predIter;
    this->fVertices.splice(predIter,
                           this->fVertices,
                           vtxIter);
  }

  this->fDirty = true;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::ActivateVertex
(Vertex* pVertex,
 unsigned int iteration,
 unsigned int numOfPoints)
{

  // change vertex attribute
  pVertex->SetLastActiveIn(iteration);
  
  // sort fInactiveVertices
  typedef ::Container<Vertex> CV;
  if (this->fInactiveVertices.iterator_to(*pVertex) != this->fInactiveVertices.end()&&
      pVertex != &this->fInactiveVertices.back() &&
      Container<Vertex>::Type::node_traits::get_next(this->fInactiveVertices.iterator_to(*pVertex).pointed_node()) != NULL)
  {
    this->fInactiveVertices.splice(this->fInactiveVertices.end(),
                                   this->fInactiveVertices,
                                   this->fInactiveVertices.iterator_to(*pVertex));
  }
  this->fDirty = true;

}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::MakeVertexActivityNeutral
(Vertex *pVertex)
{
  // determine neutral activite,
  // i.e., acitivity of last vtx in active part
  const Activity neutralActivity = this->fVertices.back().GetActivity();
  
  // assign neutral activity
  this->fTotalActivity -= pVertex->GetActivity();
  pVertex->SetActivity(neutralActivity);
  this->fTotalActivity += pVertex->GetActivity();

  // sort fVertices
  this->fVertices.splice(this->fVertices.end(),
                         this->fVertices,
                         this->fVertices.iterator_to(*pVertex));
  this->fDirty = true;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Vertex*
TriangleMesh::FindMostActiveVertex
(unsigned int numOfPoints)
const
{
  Vertex* const pVertex = const_cast<Vertex*>(&(this->fVertices.front()));
  
  return pVertex;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::FindInactiveVertices
(unsigned int currentIteration,
 Flt vertexInactivityThreshold,
 std::vector<Vertex*>* pInactiveVertices)
const
{
  const unsigned int legalInactivity =
    vertexInactivityThreshold * this->GetNumOfVertices();
  
  BOOST_FOREACH(const Vertex& vtx,
                ::AddContainer<Vertex>::ConstRange(this->fInactiveVertices))
  {
    const unsigned int currentInactivity =
      currentIteration - vtx.GetLastActiveIn();
    if (currentInactivity > legalInactivity)
    {
      pInactiveVertices->push_back(const_cast<Vertex*>(&vtx));
    }
    else
    {
      break;
    }
  }
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
unsigned int
TriangleMesh::CalculateCollapseError
(Edge* pEdge, 
 Vertex* pVtxLeft, 
 Vertex* pVtxRight)
const
{
  const int sourceValence = 
    static_cast<int>(pEdge->GetV0()->GetValence());
  const int targetValence = 
    static_cast<int>(pEdge->GetV1()->GetValence());
  const int leftValence = (pVtxLeft != NULL) ?
    static_cast<int>(pVtxLeft->GetValence()) :
    7;
  const int rightValence = (pVtxRight != NULL) ?
    static_cast<int>(pVtxRight->GetValence()):
    7;
  
  const int regErrorA = sourceValence + targetValence - 10;
  const int regErrorB = leftValence - 7;
  const int regErrorC = rightValence - 7;
  
  return static_cast<unsigned int>(
    regErrorA * regErrorA + regErrorB * regErrorB + regErrorC * regErrorC);
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
bool
TriangleMesh::EdgeFlipNecessary
(const Vertex* pStart,
 const Vertex* pEnd,
 const Vertex* pCandidate0,
 const Vertex* pCandidate1)
const
{
  Flt existingRoughness =
    this->GetRoughness(pStart, pEnd, pCandidate0, pCandidate1);
  Flt newRoughness =
    this->GetRoughness(pCandidate0, pCandidate1, pStart, pEnd);
  
  return (existingRoughness > newRoughness);
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::RepairEdges
(const std::vector<Edge*>& edgesToRepair)
{
  typedef std::vector<Vertex*> VtxContainer;
  BOOST_FOREACH(Edge* pEdgeToRepair, edgesToRepair)
  {
    if (pEdgeToRepair->GetIsActive() && (pEdgeToRepair->GetValence() != 2))
    {
      VtxContainer commonNeighbours;
      this->FindCommonNeighbours(pEdgeToRepair->GetV0(),
                                 pEdgeToRepair->GetV1(),
                                 &commonNeighbours);
      if (commonNeighbours.size() > 2)
      {
        this->SelectTwoLeastRoughNeighbours(pEdgeToRepair->GetV0(),
                                            pEdgeToRepair->GetV1(),
                                            &commonNeighbours);
      }
      
      // check if interconnecting edge is border on quadrangular hole
      if ((pEdgeToRepair->GetValence() == 1) && commonNeighbours.size() == 1)
      {
        Edge* pBorderEdges[4] = { NULL, NULL, NULL, NULL };
        Vertex* pBorderVertices[4] = { NULL, NULL, NULL, NULL };
        
        bool foundLoop = this->GetQuadHoleBorderLoop(pEdgeToRepair,
                                                     pBorderEdges,
                                                     pBorderVertices);
        if (foundLoop)
        {
          const Flt lengthA2 =
            (pBorderVertices[0]->GetPosition() -
             pBorderVertices[2]->GetPosition()).GetSquaredLength();
          const Flt lengthB2 =
            (pBorderVertices[1]->GetPosition() -
             pBorderVertices[3]->GetPosition()).GetSquaredLength();
          const Flt avgEdgeLengt2 = this->GetAverageEdgeLength();
          
          if (lengthA2 < sc<Flt>(9.0) * avgEdgeLengt2 &&
              lengthB2 < sc<Flt>(9.0) * avgEdgeLengt2)
          {
            Flt roughness02 =
            this->GetRoughness(pBorderVertices[0], pBorderVertices[2],
                               pBorderVertices[1], pBorderVertices[3]);
            Flt roughness13 =
            this->GetRoughness(pBorderVertices[1], pBorderVertices[3],
                               pBorderVertices[0], pBorderVertices[2]);
            
            if (roughness02 < roughness13)
            {
              this->NewEdge(pBorderVertices[0], pBorderVertices[2]);
              this->NewTriangle(pBorderVertices[0],
                                pBorderVertices[1],
                                pBorderVertices[2]);
              this->NewTriangle(pBorderVertices[0],
                                pBorderVertices[2],
                                pBorderVertices[3]);
            }
            else
            {
              this->NewEdge(pBorderVertices[1], pBorderVertices[3]);
              this->NewTriangle(pBorderVertices[0],
                                pBorderVertices[1],
                                pBorderVertices[3]);
              this->NewTriangle(pBorderVertices[1],
                                pBorderVertices[2],
                                pBorderVertices[3]);
            }
          }
        }
      }
      else
      {
        const Vertex* const pVtxOfTri = (pEdgeToRepair->GetT0() != NULL) ?
        pEdgeToRepair->GetT0()->GetOther(pEdgeToRepair->GetV0(),
                                         pEdgeToRepair->GetV1()) : NULL;
        BOOST_FOREACH(Vertex* pVN, commonNeighbours)
        {
          // create triangle only if it does not exist
          if (pVN != pVtxOfTri)
          {
            this->NewTriangle(pEdgeToRepair->GetV0(),
                              pEdgeToRepair->GetV1(),
                              pVN);
          }
        }
      }
    }
  }
  this->fDirty = true;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::Repair
(unsigned char c,
 unsigned int iteration)
{
  BOOST_FOREACH(Edge& edge, this->fEdges)
  {
    if (edge.GetValence() < 2)
    {
      Vertex* pVA = edge.GetV0();
      Vertex* pVB = edge.GetV1();
      std::vector<Vertex*> commonNeighbours;
      this->FindCommonNeighbours(pVA, pVB, &commonNeighbours);
      BOOST_FOREACH(Vertex* pCommonNeighbour, commonNeighbours)
      {
        bool triExists = false;
        BOOST_FOREACH(Triangle* pAdjTri, Adjacent<Triangle>::Around(&edge))
        {
          if (pAdjTri->GetOther(pVA, pVB) == pCommonNeighbour)
          {
            triExists = true;
            break;
          }
        }
        
        if (!triExists)
        {
          Triangle* pNewTri = this->NewTriangle(pVA, pVB, pCommonNeighbour);
          if (pNewTri != NULL)
          {
            std::cout << c;
            if (iteration != 0)
            {
              std::cout << " @ " << iteration << std::endl;
            }
            pNewTri->GetE0()->SetAge(static_cast<Edge::Age>(0));
            pNewTri->GetE1()->SetAge(static_cast<Edge::Age>(0));
            pNewTri->GetE2()->SetAge(static_cast<Edge::Age>(0));
          }
        }
      }
    }
  }
  this->fDirty = true;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Flt
TriangleMesh::GetRoughness
(const Vertex* const pV0,
 const Vertex* const pV1,
 const Vertex* const pVL,
 const Vertex* const pVR)
const
{
  const Position posV0(pV0->GetPosition());
  const Position posV1(pV1->GetPosition());
  const Position posVL(pVL->GetPosition());
  const Position posVR(pVR->GetPosition());
  
  const Vector normV0V1(normalize(posV1 - posV0));
  
  const Vector V0VL(posVL - posV0);
  const Vector normInLeft(
    normalize(V0VL - dotProduct(V0VL, normV0V1) * normV0V1));
  const Vector V0VR(posVR - posV0);
  const Vector normInRight(
    normalize(V0VR - dotProduct(V0VR, normV0V1) * normV0V1));
  
  return (dotProduct(normInLeft, normInRight) + sc<Flt>(1.0));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
bool
TriangleMesh::GetQuadHoleBorderLoop
(Edge* pEdge,
 Edge* pLoopEdges[4],
 Vertex* pLoopVertices[4])
{
  // exit if no edge is specified
  if (pEdge == NULL)
  {
    return false;
  }
  
  if (pEdge->GetIsActive() && (pEdge->GetValence() == 1))
  {
    Edge* pLoopEdgesRet[4] = { pEdge, NULL, NULL, NULL };
    Vertex* pLoopVerticesRet[4] =
      { pEdge->GetV0(), pEdge->GetV1(), NULL, NULL };
    
    BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pLoopVerticesRet[1]))
    {
      if ((pAdjEdge != pEdge) && (pAdjEdge->GetValence() == 1))
      {
        pLoopEdgesRet[1] = pAdjEdge;
        pLoopVerticesRet[2] = pLoopEdgesRet[1]->GetOther(pLoopVerticesRet[1]);
        
        std::vector<Vertex*> commonLoopNeighbours;
        this->FindCommonNeighbours(pLoopVerticesRet[0],
                                   pLoopVerticesRet[2],
                                   &commonLoopNeighbours);
        
        BOOST_FOREACH(Vertex* pLoopVtx, commonLoopNeighbours)
        {
          if (pLoopVtx != pLoopVerticesRet[1])
          {
            pLoopEdgesRet[2] = this->GetEdge(pLoopVerticesRet[2], pLoopVtx);
            pLoopEdgesRet[3] = this->GetEdge(pLoopVtx, pLoopVerticesRet[0]);
            
            assert(pLoopEdgesRet[2] != NULL);
            assert(pLoopEdgesRet[3] != NULL);
            
            if ((pLoopEdgesRet[2]->GetValence() == 1) &&
                (pLoopEdgesRet[3]->GetValence() == 1))
            {
              pLoopVerticesRet[3] =
                pLoopEdgesRet[2]->GetOther(pLoopVerticesRet[2]);
              
              AABB_T<Flt> loopAABB;
              loopAABB.ResizeToIncludePoint(pLoopVerticesRet[0]->GetPosition());
              loopAABB.ResizeToIncludePoint(pLoopVerticesRet[1]->GetPosition());
              loopAABB.ResizeToIncludePoint(pLoopVerticesRet[2]->GetPosition());
              loopAABB.ResizeToIncludePoint(pLoopVerticesRet[3]->GetPosition());
              
              Vertex* pLoopOtherVtx[4] =
                { pLoopEdgesRet[0]->GetT0()->GetOther(pLoopVerticesRet[0],
                                                      pLoopVerticesRet[1]),
                  pLoopEdgesRet[1]->GetT0()->GetOther(pLoopVerticesRet[1],
                                                      pLoopVerticesRet[2]),
                  pLoopEdgesRet[2]->GetT0()->GetOther(pLoopVerticesRet[2],
                                                      pLoopVerticesRet[3]),
                  pLoopEdgesRet[3]->GetT0()->GetOther(pLoopVerticesRet[3],
                                                      pLoopVerticesRet[0]) };
            
              if ((!loopAABB.Contains(pLoopOtherVtx[0]->GetPosition())) &&
                  (!loopAABB.Contains(pLoopOtherVtx[1]->GetPosition())) &&
                  (!loopAABB.Contains(pLoopOtherVtx[2]->GetPosition())) &&
                  (!loopAABB.Contains(pLoopOtherVtx[3]->GetPosition())))
              {
                pLoopEdges[0] = pLoopEdgesRet[0];
                pLoopEdges[1] = pLoopEdgesRet[1];
                pLoopEdges[2] = pLoopEdgesRet[2];
                pLoopEdges[3] = pLoopEdgesRet[3];
                
                pLoopVertices[0] = pLoopVerticesRet[0];
                pLoopVertices[1] = pLoopVerticesRet[1];
                pLoopVertices[2] = pLoopVerticesRet[2];
                pLoopVertices[3] = pLoopVerticesRet[3];
                
                return true;
              }
            }
          }
        }
      }
    }
  }
  return false;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
bool
TriangleMesh::GetConvexTriAtHole
(Edge* pEdge,
 Edge* pLoopEdges[2],
 Vertex* pLoopVertices[3])
{
  // exit if no edge is specified
  if (pEdge == NULL)
  {
    return false;
  }
  
  if (pEdge->GetIsActive() && (pEdge->GetValence() == 1))
  {
    Edge* pLoopEdgesRet[2] = { pEdge, NULL };
    Vertex* pLoopVerticesRet[3] =
      { pEdge->GetV0(), pEdge->GetV1(), NULL };
    
    BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pLoopVerticesRet[1]))
    {
      if ((pAdjEdge != pEdge) && (pAdjEdge->GetValence() == 1))
      {
        pLoopEdgesRet[1] = pAdjEdge;
        pLoopVerticesRet[2] = pLoopEdgesRet[1]->GetOther(pLoopVerticesRet[1]);
        
        if (pLoopEdgesRet[0]->GetT0() != pLoopEdgesRet[1]->GetT0())
        {
          //       2
          //      .|\
          //     . e \
          //    .  1  \
          //   .   |   \
          //  0-e0-1
          //   \   |
          //    \  |
          //     \ |
          //      \|
          //       +
          const Position pos0(pLoopVerticesRet[0]->GetPosition());
          const Position pos1(pLoopVerticesRet[1]->GetPosition());
          const Position pos2(pLoopVerticesRet[2]->GetPosition());
          
          const Vertex* const pVtxE0T0other =
            pLoopEdgesRet[0]->GetT0()->GetOther(pLoopVerticesRet[0],
                                                pLoopVerticesRet[1]);
          const Position posE0T0other(pVtxE0T0other->GetPosition());
          const Vertex* const pVtxE1T0other =
            pLoopEdgesRet[1]->GetT0()->GetOther(pLoopVerticesRet[1],
                                                pLoopVerticesRet[2]);
          const Position posE1T0other(pVtxE1T0other->GetPosition());
          
          const Vector normA = crossProduct(posE0T0other - pos0,
                                            pos1 - pos0);
          const Vector normB = crossProduct(posE1T0other - pos1,
                                            pos2 - pos1);
          const Vector normC = crossProduct(pos1 - pos0,
                                            pos2 - pos0);
          if ((dotProduct(normC, normA) >= sc<Flt>(0.0)) &&
              (dotProduct(normC, normB) >= sc<Flt>(0.0)))
          {
            if ((pos2 - pos0).GetSquaredLength() < 1.4 * 
                ((pos1 - pos0).GetSquaredLength() + (pos2 - pos1).GetSquaredLength()))
            {
              pLoopEdges[0] = pLoopEdgesRet[0];
              pLoopEdges[1] = pLoopEdgesRet[1];
              
              pLoopVertices[0] = pLoopVerticesRet[0];
              pLoopVertices[1] = pLoopVerticesRet[1];
              pLoopVertices[2] = pLoopVerticesRet[2];
              
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::LimitNumberOfTextureCandidates
()
{
#ifndef NO_TEXTURES

#endif
}






////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMesh::Info
()
const
{
  LOG_INFO("Info:");
  LOG_INFO("=====");
  LOG_INFO("Vertices:     " << this->GetNumOfVertices());
  LOG_INFO("Triangles:    " << this->GetNumOfTriangles());

  // get mean valence
  const double numOfVertices = static_cast<double>(this->GetNumOfVertices());
  const double numOfTriangles = static_cast<double>(this->GetNumOfTriangles());
  double meanValence = 0.0;
  double stdDev = 0.0;
  unsigned long valence = 0;
  
  double valence3 = 0.0;
  double valence4 = 0.0;
  double valence5 = 0.0;
  double valence6 = 0.0;
  double valence7 = 0.0;
  double valence8 = 0.0;
  double valence9 = 0.0;
  
  BOOST_FOREACH(const Vertex& vertex, this->GetVertices())
  {
    unsigned int vertexValence = 0;
    vertexValence = vertex.GetValence();
    
    valence += vertexValence;
    if (vertexValence == 3)
    {
      valence3 += 1.0;
    }
    if (vertexValence == 4)
    {
      valence4 += 1.0;
    }
    if (vertexValence == 5)
    {
      valence5 += 1.0;
    }
    if (vertexValence == 6)
    {
      valence6 += 1.0;
    }
    if (vertexValence == 7)
    {
      valence7 += 1.0;
    }
    else if (vertexValence == 8)
    {
      valence8 += 1.0;
    }
    else if (vertexValence == 9)
    {
      valence9 += 1.0;
    }
  }
  meanValence = static_cast<double>(valence) / numOfVertices;
  valence3 /= numOfVertices;
  valence4 /= numOfVertices;
  valence5 /= numOfVertices;
  valence6 /= numOfVertices;
  valence7 /= numOfVertices;
  valence8 /= numOfVertices;
  valence9 /= numOfVertices;

  BOOST_FOREACH(const Vertex& vertex, this->GetVertices())
  {
    stdDev += (static_cast<double>(vertex.GetValence()) - meanValence) *
    (static_cast<double>(vertex.GetValence()) - meanValence);
  }
  stdDev /= numOfVertices - 1.0;
  
  stdDev = sqrt(stdDev);
  LOG_INFO("Avg. valence: " << meanValence);
  LOG_INFO("Std. dev:     " << stdDev);
  LOG_INFO("Valence 3  :  " << (100.0 * valence3) << "%");
  LOG_INFO("Valence 4  :  " << (100.0 * valence4) << "%");
  LOG_INFO("Valence 5  :  " << (100.0 * valence5) << "%");
  LOG_INFO("Valence 6  :  " << (100.0 * valence6) << "%");
  LOG_INFO("Valence 7  :  " << (100.0 * valence7) << "%");
  LOG_INFO("Valence 8  :  " << (100.0 * valence8) << "%");
  LOG_INFO("Valence 9  :  " << (100.0 * valence9) << "%");

  LOG_INFO("Valence 4-7:  " << (100.0 * (valence4 + valence5 + valence6 + valence7)) << "%");
  
  
  // get mean triangle area
  Flt regularity = 0.0;
  BOOST_FOREACH(const Triangle& tri, this->GetTriangles())
  {
    const Position v0 = tri.GetV0()->GetPosition();
    const Position v1 = tri.GetV1()->GetPosition();
    const Position v2 = tri.GetV2()->GetPosition();
    
    const Position barycenter = (v0 + v1 + v2) * sc<Flt>(1.0 / 3.0);
    
    const Flt sqV0B = (v0 - barycenter).GetSquaredLength();
    const Flt sqV1B = (v1 - barycenter).GetSquaredLength();
    const Flt sqV2B = (v2 - barycenter).GetSquaredLength();
    
    Flt sqMin = sqV0B < sqV1B ? (sqV0B < sqV2B ? sqV0B : sqV2B) : (sqV1B < sqV2B ? sqV1B : sqV2B);
    Flt sqMax = sqV0B > sqV1B ? (sqV0B > sqV2B ? sqV0B : sqV2B) : (sqV1B > sqV2B ? sqV1B : sqV2B);
    
    regularity += sqrtf(sqMin) / sqrtf(sqMax);
  }
  LOG_INFO("Regularity : " << regularity / numOfTriangles);
  
  unsigned int bmuCount = 0u;
  BOOST_FOREACH(const Vertex& vertex, this->GetVertices())
  {
    bmuCount += vertex.GetBmuCount();
  }
  LOG_INFO("Sum BMU count:   " << bmuCount);
  LOG_INFO("Total BMU count: " << this->fTotalBmuCount);
  
  
#ifndef NO_TEXTURES
  unsigned int minNumOfTexturesPerVertex = 0u - 1u;
  unsigned int maxNumOfTexturesPerVertex = 0u;
  unsigned int totalNumberOfTextures = 0u;
  
  BOOST_FOREACH(const Vertex& vtx, this->GetVertices())
  {
    unsigned int count = 0u;
    BOOST_FOREACH(const TextureCoordinate& texCoord,
                  vtx.GetTextureCoordinates())
    {
      UNUSED(texCoord);
      ++count;
    }
    minNumOfTexturesPerVertex = std::min(count, minNumOfTexturesPerVertex);
    maxNumOfTexturesPerVertex = std::max(count, maxNumOfTexturesPerVertex);
    totalNumberOfTextures += count;
  }
  
  const float avgTexturesPerVertex =
    static_cast<float>(totalNumberOfTextures) /
    static_cast<float>(this->GetNumOfVertices());
  
  unsigned int standardDeviationAccumulator = 0u;
  BOOST_FOREACH(const Vertex& vtx, this->GetVertices())
  {
    unsigned int count = 0u;
    BOOST_FOREACH(const TextureCoordinate& texCoord,
                  vtx.GetTextureCoordinates())
    {
      UNUSED(texCoord);
      ++count;
    }
    standardDeviationAccumulator +=
      (avgTexturesPerVertex - count) * (avgTexturesPerVertex - count);
  }
  
  const float standardDeviation =
    sqrtf(static_cast<float>(standardDeviationAccumulator) /
          static_cast<float>(this->GetNumOfVertices() - 1));
  
  LOG_INFO("Min. tex. cand. per vtx.: " << minNumOfTexturesPerVertex);
  LOG_INFO("Max. tex. cand. per vtx.: " << maxNumOfTexturesPerVertex);
  LOG_INFO("Avg. tex. cand. per vtx.: " << avgTexturesPerVertex);
  LOG_INFO("Stddev.                 : " << standardDeviation);
#endif
}

