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

#ifndef SURFACE_RECONSTRUCTION__TRIANGLE_MESH_H_
#define SURFACE_RECONSTRUCTION__TRIANGLE_MESH_H_


#ifndef Q_MOC_RUN
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#endif

#include <common/unused.h>

#include "container.h"
#include "data_types.h"
#include "edge.h"
#include "octree.h"
#include "point.h"
#include "triangle.h"
#include "vertex.h"
#include "io/data_storage.h"




////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
class TriangleMesh : boost::noncopyable
{
public:
  typedef boost::shared_ptr<TriangleMesh> SharedPtr;
  
  TriangleMesh();
  ~TriangleMesh();
  
  void ResetVerticesEdgesTriangles();
  
  /// \brief Stores a triangle mesh into a ply file
  void Save(const io::DataStorage<TriangleMesh>& dataStorage);
  
  Triangle* NewTriangle(Vertex* pV0, Vertex* pV1, Vertex* pV2);
  Edge* NewEdge(Vertex* pV0, Vertex* pV1);
  
  template <typename Tx, typename Ty, typename Tz>
  Vertex* NewVertex(Tx x, Ty y, Tz z)
  {
    return this->NewVertex(Position(sc<Flt>(x), sc<Flt>(y), sc<Flt>(z)));
  }
  
  Vertex* NewVertex(const Position& position)
  {
    Vertex* pV = this->NewVertex();
    pV->SetPosition(position);
    this->fpOctree->Insert(pV);
    return pV;
  }
  
  void DeleteTriangle(Triangle* pTriangle,
                      std::vector<Edge*>* pEdgesToRepair = NULL);
  void DeleteEdge(Edge* pEdge, bool deleteDanglingVertices = true,
                  std::vector<Edge*>* pEdgesToRepair = NULL);
  void DeleteVertex(Vertex* pVertex);
  
  void DeleteAllTriangles();
  void DeleteAllEdges();
  void DeleteAllVertices();
  
  ::Container<Triangle>::SizeType GetNumOfTriangles() const
  {
    return this->fTriangles.size();
  }

  ::Container<Edge>::SizeType GetNumOfEdges() const
  {
    return this->fEdges.size();
  }
  
  ::Container<Vertex>::SizeType GetNumOfVertices() const
  {
    return this->fVertices.size();
  }

  ::Container<Triangle>::Range GetTriangles() { return this->fTriangles; }
  ::Container<Triangle>::ConstRange GetTriangles() const { return this->fTriangles; }
  
  ::Container<Edge>::Range GetEdges() { return this->fEdges; }
  ::Container<Edge>::ConstRange GetEdges() const { return this->fEdges; }
  
  ::Container<Vertex>::Range GetVertices() { return this->fVertices; }
  ::Container<Vertex>::ConstRange GetVertices() const { return this->fVertices; }
  
  void FindClosestVertices(const Position& position,
                           Vertex* closestVertices[2]);
  
  void MoveVertex(Vertex* pVertex, const Vector& movement)
  {
    const Position newPosition(pVertex->GetPosition() + movement);
    
    if (pVertex->GetParentOctreeNode() != NULL &&
        pVertex->GetParentOctreeNode()->Contains(newPosition))
    {
      pVertex->SetPosition(newPosition);
    }
    else if (pVertex->GetParentOctreeNode() != NULL)
    {
      bool success = this->fpOctree->RemoveElement(pVertex);
      assert(success);
      UNUSED(success);
      
      pVertex->SetPosition(newPosition);
      
      this->fpOctree->Insert(pVertex);
    }
    else
    {
      pVertex->SetPosition(newPosition);
      this->fpOctree->Insert(pVertex);
    }
    this->fDirty = true;
  }
  
  Edge* GetEdge(Vertex* pVA, Vertex* pVB)
  {
    const Vertex* const pV0 = 
      (pVA->GetValence() <= pVB->GetValence()) ? pVA : pVB;
    BOOST_FOREACH(Edge* pEdge, Adjacent<Edge>::Around(pV0))
    {
      assert(pEdge->GetIsActive());
      if (pEdge->Connects(pVA, pVB))
      {
        return pEdge;
      }
    }
    return NULL;
  }
  
  Triangle* GetTriangle(Vertex* pV0, Vertex* pV1, Vertex* pV2)
  {
    BOOST_FOREACH(Triangle& tri, this->GetTriangles())
    {
      // find interconnecting triangle
      if ((tri.GetV0() == pV0 ||
           tri.GetV1() == pV0 ||
           tri.GetV2() == pV0) &&
          (tri.GetV0() == pV1 ||
           tri.GetV1() == pV1 ||
           tri.GetV2() == pV1) &&
          (tri.GetV0() == pV2 ||
           tri.GetV1() == pV2 ||
           tri.GetV2() == pV2))
      {
        return &tri;
      }
    }
    return NULL;
  }
  
  void FindCommonNeighbours(Vertex* pVA, 
                            Vertex* pVB, 
                            std::vector<Vertex*>* pCommonNeighbours);
  void SelectTwoLeastRoughNeighbours(Vertex* pVtxA, Vertex* pVtxB,
                                     std::vector<Vertex*>* pCommonNeighbours);
  
  void IncrementVertexActivity(Vertex* pVertex, unsigned int numOfPoints);
  void ActivateVertex(Vertex* pVertex, unsigned int iteration,
                      unsigned int numOfPoints);
  void MakeVertexActivityNeutral(Vertex *pVertex);
  
  void IncrementBmuCount(Vertex* pVertex)
  {
    pVertex->IncrementBmuCount();
    ++(this->fTotalBmuCount);
  }
  
  unsigned int GetPointsForVertex(Vertex* pVertex, unsigned int numOfPoints)
  const
  {
    const Flt fraction =
      sc<Flt>(pVertex->GetBmuCount()) / sc<Flt>(this->fTotalBmuCount);
    return sc<unsigned int>(fraction * numOfPoints);
  }

  void DistributeBmuCountOnSplit(Vertex* pVSrcA, Vertex* pVSrcB,
                                 Vertex* pVSrcL, Vertex* pVSrcR,
                                 Vertex* pVDest)
  {
    const unsigned int vtxCount =
      2u + ((pVSrcL != NULL) ? 1u : 0u) + ((pVSrcR != NULL) ? 1u : 0u);
    
    unsigned int toDest = 0u;
    const unsigned int fromAtoDest = pVSrcA->GetBmuCount() / vtxCount;
    pVSrcA->SetBmuCount(pVSrcA->GetBmuCount() - fromAtoDest);
    toDest += fromAtoDest;
    
    const unsigned int fromBtoDest = pVSrcB->GetBmuCount() / vtxCount;
    pVSrcB->SetBmuCount(pVSrcB->GetBmuCount() - fromBtoDest);
    toDest += fromBtoDest;
    
    if (pVSrcL != NULL)
    {
      const unsigned int fromLtoDest = pVSrcL->GetBmuCount() / vtxCount;
      pVSrcL->SetBmuCount(pVSrcL->GetBmuCount() - fromLtoDest);
      toDest += fromLtoDest;
    }
    
    if (pVSrcR != NULL)
    {
      const unsigned int fromRtoDest = pVSrcR->GetBmuCount() / vtxCount;
      pVSrcR->SetBmuCount(pVSrcR->GetBmuCount() - fromRtoDest);
      toDest += fromRtoDest;
    }
    
    pVDest->SetBmuCount(toDest);
  }
  
  void DistributeBmuCountOnCollapse(Vertex* pVtxSource, Vertex* pVtxTarget,
                                    Vertex* pVtxLeft, Vertex* pVtxRight)
  {
    unsigned int count = 1u;
    if (pVtxLeft != NULL)
    {
      ++count;
    }
    if (pVtxRight != NULL)
    {
      ++count;
    }
    const unsigned int quotient = pVtxSource->GetBmuCount() / count;
    const unsigned int remainder = pVtxSource->GetBmuCount() % count;
    
    pVtxSource->SetBmuCount(0u);
    pVtxTarget->SetBmuCount(pVtxTarget->GetBmuCount() + quotient + remainder);
    if (pVtxLeft != NULL)
    {
      pVtxLeft->SetBmuCount(pVtxLeft->GetBmuCount() + quotient);
    }
    if (pVtxRight != NULL)
    {
      pVtxRight->SetBmuCount(pVtxRight->GetBmuCount() + quotient);
    }
  }
  
  
  Vertex* FindMostActiveVertex(unsigned int numberOfPoints) const;
  void FindInactiveVertices(unsigned int currentIteration,
                            Flt vertexInactivityThreshold,
                            std::vector<Vertex*>* pInactiveVertices) const;
  
  unsigned int CalculateCollapseError (Edge* pEdge, 
                                       Vertex* pVtxLeft, 
                                       Vertex* pVtxRight) const;
  
  bool EdgeFlipNecessary(const Vertex* pStart,
                         const Vertex* pEnd,
                         const Vertex* pCandidate0,
                         const Vertex* pCandidate1) const;
  
  void RepairEdges(const std::vector<Edge*>& edgesToRepair);
  
  void Repair(unsigned char c, unsigned int iteration = 0);
  
  /// \returns  squared roughness of the triangles adjacent to edge pV0-pV1
  /// \param    pV0   beginning of the edge
  /// \param    pV1   end of the edge
  /// \param    pVL   vertex defining the left triangle pVL-pV0-pV1
  /// \param    pVR   vertex defining the right triangle pVR-pV1-pV0
  Flt GetRoughness(const Vertex* const pV0,
                                         const Vertex* const pV1,
                                         const Vertex* const pVL,
                                         const Vertex* const pVR) const;
  
  bool GetQuadHoleBorderLoop(Edge* pEdge,
                             Edge* pLoopEdges[4],
                             Vertex* pLoopVertices[4]);
  
  bool GetConvexTriAtHole(Edge* pEdge,
                          Edge* pLoopEdges[2],
                          Vertex* pLoopVertices[3]);
  
  void Info() const;
  
  bool IsDirty() const { return this->fDirty; }
  void RemoveDirty() { this->fDirty = false; }
  
  void LimitNumberOfTextureCandidates();
  
  float GetAverageEdgeLength() const { return this->fAverageEdgeLength; }
  void SetAverageEdgeLength(float value) { this->fAverageEdgeLength = value; }
  
private:
  Vertex* NewVertex();
  
  void AllocateTriangles(unsigned int num);
  void AllocateEdges(unsigned int num);
  void AllocateVertices(unsigned int num);
  
  ::Container<Triangle>::Type fTriangles;
  ::Container<Triangle>::Type fUnusedTriangles;
  
  ::Container<Edge>::Type fEdges;
  ::Container<Edge>::Type fUnusedEdges;
  
  ::Container<Vertex>::Type fVertices;
  ::AddContainer<Vertex>::Type fInactiveVertices;
  ::Container<Vertex>::Type fUnusedVertices;
  ::Container<Vertex>::Iterator fInactiveVerticesBegin;
  
  Activity fTotalActivity;
  unsigned int fTotalBmuCount;

  Octree* fpOctree;
  bool fDirty;
  
  unsigned int fPointsPerVertex;
  
  float fAverageEdgeLength;
};  // class TriangleMesh


#endif  // #ifndef SURFACE_RECONSTRUCTION__TRIANGLE_MESH_H_
