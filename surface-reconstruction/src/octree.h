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

#ifndef SURFACE_RECONSTRUCTION__OCTREE_H_
#define SURFACE_RECONSTRUCTION__OCTREE_H_

#include "data_types.h"
#include "octree_node.h"
#include "vertex.h"


class Octree
{
public:
  Octree(const Position& minCorner,
         const Position& maxCorner)
  : fpRoot(new OctreeNode(minCorner, maxCorner, NULL))
  {}
  
  void Insert(Vertex* pElement)
  {
    if (this->fpRoot != NULL && this->fpRoot->Contains(pElement->GetPosition()))
    {
      this->fpRoot->Insert(pElement);
    }
    else
    {
      this->GrowToInclude(pElement);
      this->Insert(pElement);
    }
  }
  
  bool RemoveElement(Vertex* pElement)
  {
    OctreeNode* const pElementsParent = pElement->GetParentOctreeNode();
    assert(pElementsParent != NULL);
    assert(pElementsParent->IsLeaf());
    
    // remove element from its octree node
    const bool removed = pElementsParent->RemoveElement(pElement);
    assert(removed);
    
    // check if subsequent parent nodes need to be collapsed
    OctreeNode* pParentNode = pElementsParent->GetParentNode();
    while((pParentNode != NULL) && (pParentNode->NeedsToBeCollapsed()))
    {
      pParentNode->Collapse();
      pParentNode = pParentNode->GetParentNode();
    }
    
    return removed;
  }
  
  Vertex* FindNearestNeighbour(const Position& position)
  {
    Vertex* pFound = NULL;
    Flt minSquaredDistance =
      std::numeric_limits<Flt>::infinity();
    if (this->fpRoot != NULL)
    {
      this->fpRoot->FindNearestNeighbour(position,
                                         &pFound,
                                         &minSquaredDistance);
    }
    return pFound;
  }
  
  void FindTwoNearestNeighbour(const Position& position,
                               Vertex** ppNearestNeighbour0,
                               Flt* pMinSquaredDistance0,
                               Vertex** ppNearestNeighbour1,
                               Flt* pMinSquaredDistance1)
  {
    *ppNearestNeighbour0 = NULL;
    *pMinSquaredDistance0 =
      std::numeric_limits<Flt>::infinity();
    *ppNearestNeighbour1 = NULL;
    *pMinSquaredDistance1 =
      std::numeric_limits<Flt>::infinity();
    
    if (this->fpRoot != NULL)
    {
      this->fpRoot->FindNearestNeighbour(position,
                                         ppNearestNeighbour0,
                                         pMinSquaredDistance0,
                                         ppNearestNeighbour1,
                                         pMinSquaredDistance1);
    }
  }
  
  OctreeNode* GetRoot() { return this->fpRoot; }
  
private:
  void GrowToInclude(Vertex* pElement)
  {
    const Vector centerToElement =
      pElement->GetPosition() - fpRoot->GetCenter();
    
    unsigned int index = 
      OctreeNode::kLeft | OctreeNode::kBottom | OctreeNode::kNear;
    
    const Position oldMinCorner = fpRoot->GetMinCorner();
    const Position oldMaxCorner = fpRoot->GetMaxCorner();
    Position newMinCorner;
    Position newMaxCorner;
    
    // Compute bounds of the new root node.
    if (centerToElement.GetX() < 0)
    {
      newMinCorner.SetX(2 * oldMinCorner.GetX() - oldMaxCorner.GetX());
      newMaxCorner.SetX(oldMaxCorner.GetX());
      
      // The current root will be in the right half of the new root.
      index |= OctreeNode::kRight;
    }
    else
    {
      newMinCorner.SetX(oldMinCorner.GetX());
      newMaxCorner.SetX(2 * oldMaxCorner.GetX() - oldMinCorner.GetX());
    }
    
    if (centerToElement.GetY() < 0)
    {
      newMinCorner.SetY(2 * oldMinCorner.GetY() - oldMaxCorner.GetY());
      newMaxCorner.SetY(oldMaxCorner.GetY());
      
      // The current root will be in the top half of the new root.
      index |= OctreeNode::kTop;
    }
    else
    {
      newMinCorner.SetY(oldMinCorner.GetY());
      newMaxCorner.SetY(2 * oldMaxCorner.GetY() - oldMinCorner.GetY());
    }
    
    if (centerToElement.GetZ() < 0)
    {
      newMinCorner.SetZ(2 * oldMinCorner.GetZ() - oldMaxCorner.GetZ());
      newMaxCorner.SetZ(oldMaxCorner.GetZ());
      
      // The current root will be in the back half of the new root.
      index |= OctreeNode::kFar;
    }
    else
    {
      newMinCorner.SetZ(oldMinCorner.GetZ());
      newMaxCorner.SetZ(2 * oldMaxCorner.GetZ() - oldMinCorner.GetZ());
    }
    
    // Create the new root node.
    OctreeNode* pNewRoot(new OctreeNode(newMinCorner, newMaxCorner, NULL));
    
    // Split the new root node, then replace one of its children by the old root.
    pNewRoot->SplitNode();
    pNewRoot->SwapChild(index, &this->fpRoot);
    // Note: fpRoot now points to the replaced empty child node.
    
    // Set the new root node as the Octree's root node.
    std::swap(this->fpRoot, pNewRoot);
    // Note: By swapping fpRoot and pNewRoot, pNewRoot now points to the empty
    // child node
    delete pNewRoot;
  }
  
  OctreeNode* fpRoot;
};  // class Octree

  
#endif  // #ifndef SURFACE_RECONSTRUCTION__OCTREE_H_
