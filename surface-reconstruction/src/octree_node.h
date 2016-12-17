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

#ifndef SURFACE_RECONSTRUCTION__OCTREE_NODE_H_
#define SURFACE_RECONSTRUCTION__OCTREE_NODE_H_

#include <limits>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#endif

#include <common/aabb.h>
#include <common/vec3.h>

#include "data_types.h"
#include "vertex.h"

class OctreeNode : public AABB_T<Flt>
{
public:
  // Bits in indices:
  // xx0 - left     xx1 - right
  // x0x - bottom   x1x - top
  // 0xx - near     1xx - far
  enum ChildIndex
  {
    kLeft   = 0x0,
    kRight  = 0x1,
    kBottom = 0x0 << 1,
    kTop    = 0x1 << 1,
    kNear   = 0x0 << 2,
    kFar    = 0x1 << 2
  };
  
  OctreeNode(const Position& minCorner,
             const Position& maxCorner,
             OctreeNode* pParent)
  : AABB_T<Flt>(minCorner, maxCorner)
  , fpParent(pParent)
  {
    this->fpChildren[0] = NULL;
    this->fpChildren[1] = NULL;
    this->fpChildren[2] = NULL;
    this->fpChildren[3] = NULL;
    this->fpChildren[4] = NULL;
    this->fpChildren[5] = NULL;
    this->fpChildren[6] = NULL;
    this->fpChildren[7] = NULL;
  }
  
  void Insert(Vertex* pElement)
  {
    if (this->IsLeaf() && (this->fLeafElements.size() < 8))
    {
      this->fLeafElements.push_back(pElement);
      pElement->SetParentOctreeNode(this);
    }
    else if (this->IsLeaf() && (this->fLeafElements.size() == 8))
    {
      this->SplitNode();
      InsertInChildren(pElement);
    }
    else if ((!this->IsLeaf()))
    {
      InsertInChildren(pElement);
    }
    else
    {
      assert(false);
    }
  }
  
  
  void SplitNode()
  {
    typedef Position Pos;
    const Position minC(this->GetMinCorner());
    const Position cent(this->GetCenter());
    const Position maxC(this->GetMaxCorner());
    
    this->fpChildren[kLeft  | kBottom | kNear] = 
    new OctreeNode(Pos(minC.GetX(), minC.GetY(), minC.GetZ()),
                   Pos(cent.GetX(), cent.GetY(), cent.GetZ()),
                   this);
    this->fpChildren[kRight | kBottom | kNear] = 
    new OctreeNode(Pos(cent.GetX(), minC.GetY(), minC.GetZ()),
                   Pos(maxC.GetX(), cent.GetY(), cent.GetZ()),
                   this);
    this->fpChildren[kLeft  | kTop    | kNear] = 
    new OctreeNode(Pos(minC.GetX(), cent.GetY(), minC.GetZ()), 
                   Pos(cent.GetX(), maxC.GetY(), cent.GetZ()),
                   this);
    this->fpChildren[kRight | kTop    | kNear] = 
    new OctreeNode(Pos(cent.GetX(), cent.GetY(), minC.GetZ()),
                   Pos(maxC.GetX(), maxC.GetY(), cent.GetZ()),
                   this);
    
    this->fpChildren[kLeft  | kBottom | kFar ] = 
    new OctreeNode(Pos(minC.GetX(), minC.GetY(), cent.GetZ()),
                   Pos(cent.GetX(), cent.GetY(), maxC.GetZ()),
                   this);
    this->fpChildren[kRight | kBottom | kFar ] = 
    new OctreeNode(Pos(cent.GetX(), minC.GetY(), cent.GetZ()),
                   Pos(maxC.GetX(), cent.GetY(), maxC.GetZ()),
                   this);
    this->fpChildren[kLeft  | kTop    | kFar ] = 
    new OctreeNode(Pos(minC.GetX(), cent.GetY(), cent.GetZ()), 
                   Pos(cent.GetX(), maxC.GetY(), maxC.GetZ()),
                   this);
    this->fpChildren[kRight | kTop    | kFar ] = 
    new OctreeNode(Pos(cent.GetX(), cent.GetY(), cent.GetZ()),
                   Pos(maxC.GetX(), maxC.GetY(), maxC.GetZ()),
                   this);
    
    if (this->fLeafElements.size() == 8)
    {
      this->InsertInChildren(this->fLeafElements[0]);
      this->InsertInChildren(this->fLeafElements[1]);
      this->InsertInChildren(this->fLeafElements[2]);
      this->InsertInChildren(this->fLeafElements[3]);
      this->InsertInChildren(this->fLeafElements[4]);
      this->InsertInChildren(this->fLeafElements[5]);
      this->InsertInChildren(this->fLeafElements[6]);
      this->InsertInChildren(this->fLeafElements[7]);
      this->fLeafElements.clear();
    }
    else if (this->fLeafElements.size() != 0)
    {
      // only empty and full nodes may be split
      assert(false);
    }
  }
  
  
  void InsertInChildren(Vertex* pElement)
  {
    const Position elementPosition(pElement->GetPosition());
    if (this->fpChildren[0]->Contains(elementPosition))
    {
      this->fpChildren[0]->Insert(pElement);
    }
    else if (this->fpChildren[1]->Contains(elementPosition))
    {
      this->fpChildren[1]->Insert(pElement);
    }
    else if (this->fpChildren[2]->Contains(elementPosition))
    {
      this->fpChildren[2]->Insert(pElement);
    }
    else if (this->fpChildren[3]->Contains(elementPosition))
    {
      this->fpChildren[3]->Insert(pElement);
    }
    else if (this->fpChildren[4]->Contains(elementPosition))
    {
      this->fpChildren[4]->Insert(pElement);
    }
    else if (this->fpChildren[5]->Contains(elementPosition))
    {
      this->fpChildren[5]->Insert(pElement);
    }
    else if (this->fpChildren[6]->Contains(elementPosition))
    {
      this->fpChildren[6]->Insert(pElement);
    }
    else if (this->fpChildren[7]->Contains(elementPosition))
    {
      this->fpChildren[7]->Insert(pElement);
    }
    else
    {
      assert(false);
    }
  }
  
  bool RemoveElement(Vertex* pElement)
  {
    std::vector<Vertex*>::iterator iter =
    std::find(this->fLeafElements.begin(),
              this->fLeafElements.end(),
              pElement);
    
    if (iter != this->fLeafElements.end())
    {
      this->fLeafElements.erase(iter);
      pElement->SetParentOctreeNode(NULL);
      return true;
    }
    return false;
  }
  
  
  
  bool NeedsToBeCollapsed()
  {
    if (this->IsLeaf())
    {
      return false;
    }
    
    bool collapsable = true;
    unsigned int totalNumOfElements = 0;
    for (unsigned int i = 0; i < 8; ++i)
    {
      if (this->fpChildren[i]->IsLeaf())
      {
        totalNumOfElements += this->fpChildren[i]->fLeafElements.size();
      }
      else
      {
        collapsable = false;
        break;
      }
    }
    
    return (collapsable && (totalNumOfElements < 9));
  }
  
  
  
  void Collapse()
  {
    for (unsigned int i = 0; i < 8; ++i)
    {
      this->fLeafElements.insert(this->fLeafElements.end(),
                                 this->fpChildren[i]->fLeafElements.begin(),
                                 this->fpChildren[i]->fLeafElements.end());
      delete this->fpChildren[i];
      this->fpChildren[i] = NULL;
    }
    BOOST_FOREACH(Vertex* pVtx, this->fLeafElements)
    {
      pVtx->SetParentOctreeNode(this);
    }
  }
  
  
  
  void FindNearestNeighbour(const Position& position, 
                            Vertex** ppNearestNeighbour,
                            Flt* pMinSquaredDistance)
  {
    if (this->IsLeaf())
    {
      // find closest in element list
      BOOST_FOREACH(Vertex* pV, this->fLeafElements)
      {
        const Flt currentSquaredDistance =
          (pV->GetPosition() - position).GetSquaredLength();
        if (currentSquaredDistance < *pMinSquaredDistance)
        {
          *pMinSquaredDistance = currentSquaredDistance;
          *ppNearestNeighbour = pV;
        }
      }
    }
    else
    {
      unsigned int childIndex = kLeft | kBottom | kNear;
      if (position.GetX() > this->fpChildren[0]->GetMaxCorner().GetX())
      {
        childIndex |= kRight;
      }
      if (position.GetY() > this->fpChildren[0]->GetMaxCorner().GetY())
      {
        childIndex |= kTop;
      }
      if (position.GetZ() > this->fpChildren[0]->GetMaxCorner().GetZ())
      {
        childIndex |= kFar;
      }
      this->fpChildren[childIndex]->FindNearestNeighbour(position, 
                                                         ppNearestNeighbour, 
                                                         pMinSquaredDistance);
      
      for (unsigned int i = 0; i < 8; ++i)
      {
        if (childIndex != i &&
            this->fpChildren[i]->IntersectsBoundingSphere(position, 
                                                          *pMinSquaredDistance))
        {
          this->fpChildren[i]->FindNearestNeighbour(position, 
                                                    ppNearestNeighbour, 
                                                    pMinSquaredDistance);
        }
      }
    }
  }
  
  bool FindNearestNeighbour(const Position& position,
                            Vertex** ppNearestNeighbour0,
                            Flt* pMinSquaredDistance0, 
                            Vertex** ppNearestNeighbour1,
                            Flt* pMinSquaredDistance1)
  {
    if (this->IsLeaf())
    {
      // find closest in element list
      BOOST_FOREACH(Vertex* pV, this->fLeafElements)
      {
        const Flt currentSquaredDistance =
        (pV->GetPosition() - position).GetSquaredLength();
        
        if (currentSquaredDistance < *pMinSquaredDistance0)
        {
          *pMinSquaredDistance1 = *pMinSquaredDistance0;
          *pMinSquaredDistance0 = currentSquaredDistance;
          
          *ppNearestNeighbour1 = *ppNearestNeighbour0;
          *ppNearestNeighbour0 = pV;
        }
        else if (currentSquaredDistance < *pMinSquaredDistance1)
        {
          *pMinSquaredDistance1 = currentSquaredDistance;
          *ppNearestNeighbour1 = pV;
        }
      }
    }
    else
    {
      unsigned int childIndex = kLeft | kBottom | kNear;
      if (position.GetX() > this->fpChildren[0]->GetMaxCorner().GetX())
      {
        childIndex |= kRight;
      }
      if (position.GetY() > this->fpChildren[0]->GetMaxCorner().GetY())
      {
        childIndex |= kTop;
      }
      if (position.GetZ() > this->fpChildren[0]->GetMaxCorner().GetZ())
      {
        childIndex |= kFar;
      }
      bool done = this->fpChildren[childIndex]->FindNearestNeighbour(
        position,
        ppNearestNeighbour0,
        pMinSquaredDistance0, 
        ppNearestNeighbour1, 
        pMinSquaredDistance1);
      if (done)
      {
        return true;
      }
      
      
      if (this->fpChildren[childIndex]->Contains(position))
      {
        const Position minCorner = this->fpChildren[childIndex]->GetMinCorner();
        const Position maxCorner = this->fpChildren[childIndex]->GetMaxCorner();
        const Vector minComponents(Vector::MinComponents(position - minCorner,
                                                         maxCorner - position));
        const Flt minDistance = std::min(minComponents.GetX(),
                                         std::min(minComponents.GetY(),
                                                  minComponents.GetZ()));
        if ((minDistance * minDistance) > *pMinSquaredDistance1)
        {
          return true;
        }
      }
      
      
      for (unsigned int i = 0; i < 8; ++i)
      {
        if (childIndex != i &&
            this->fpChildren[i]->IntersectsBoundingSphere(position, 
                                                          *pMinSquaredDistance1))
        {
          this->fpChildren[i]->FindNearestNeighbour(position,
                                                    ppNearestNeighbour0,
                                                    pMinSquaredDistance0,
                                                    ppNearestNeighbour1,
                                                    pMinSquaredDistance1);
        }
      }
    }
    return false;
  }
  
  bool IsLeaf() const
  {
    return ((fpChildren[0] == NULL) && (fpChildren[1] == NULL) && 
            (fpChildren[2] == NULL) && (fpChildren[3] == NULL) && 
            (fpChildren[4] == NULL) && (fpChildren[5] == NULL) && 
            (fpChildren[6] == NULL) && (fpChildren[7] == NULL));
  }
  
  OctreeNode* GetChild(int index) { return this->fpChildren[index]; }
  
  void SwapChild(unsigned int index, OctreeNode** ppNewChild)
  {
    OctreeNode* pNewChildsParent = (*ppNewChild)->fpParent;
    std::swap(this->fpChildren[index], *ppNewChild);
    this->fpChildren[index]->fpParent = this;
    (*ppNewChild)->fpParent = pNewChildsParent;
  }
  
  std::vector<Vertex*>::size_type NumOfLeafElements() const
  {
    return this->fLeafElements.size();
  }
  
  std::vector<Vertex*>::const_iterator LeafElementsBegin() const
  {
    return this->fLeafElements.begin();
  }

  std::vector<Vertex*>::const_iterator LeafElementsEnd() const
  {
    return this->fLeafElements.end();
  }
  
  OctreeNode* GetParentNode() const { return this->fpParent; }
  
private:
  OctreeNode* fpChildren[8];
  OctreeNode* fpParent;
  std::vector<Vertex*> fLeafElements;
};  // class OctreeNode

#endif  // #ifndef SURFACE_RECONSTRUCTION__OCTREE_NODE_H_
