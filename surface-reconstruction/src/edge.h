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

#ifndef SURFACE_RECONSTRUCTION__EDGE_H_
#define SURFACE_RECONSTRUCTION__EDGE_H_


#ifndef Q_MOC_RUN
#include <boost/noncopyable.hpp>
#endif

#include "adjacency.h"
#include "container.h"
#include "data_types.h"
#include "vertex.h"


// forward declaration
class Triangle;


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
class Edge : boost::noncopyable, public AdjacencyContainer<Triangle>
{
  CONTAINER_ELEMENT<Edge>;

public:
  friend class TriangleMesh;
  
  typedef unsigned int Age;
  
  Vertex* GetV0() const { return fpV0; }
  Vertex* GetV1() const { return fpV1; }
  Vertex* GetOther(const Vertex* const pV) const
  {
    assert((this->fpV0 == pV) || (this->fpV1 == pV));
    return ((pV == this->fpV0) ? this->fpV1 : this->fpV0);
  }
  
  Age GetAge() const { return this->fAge; }
  void SetAge(Age age) { this->fAge = age; }
  void IncrementAge() { ++this->fAge; }
  bool IsOlderThan(Flt ageThreshold) const
  {
    return this->GetAge() > ageThreshold;
  }
  
  bool Connects(Vertex* pV0, Vertex* pV1)
  {
    return (((this->fpV0 == pV0) && (this->fpV1 == pV1)) ||
            ((this->fpV0 == pV1) && (this->fpV1 == pV0)));
  }
  
  bool IsFaceless() const 
  { 
    return AdjacencyContainer<Triangle>::fElements.size() == 0; 
  }
  
  unsigned int GetValence() const 
  { 
    return AdjacencyContainer<Triangle>::fElements.size(); 
  }
  
  Flt GetSquaredLength() const
  {
    return (this->fpV0->GetPosition() - 
            this->fpV1->GetPosition()).GetSquaredLength();
  }
  
  bool IsLongerThan(Flt squaredThreshold) const
  {
    return this->GetSquaredLength() > squaredThreshold;
  }
  
  void ReplaceVertex(Vertex* pExistingVtx, Vertex* pNewVtx)
  {
    Vertex** ppVtxToBeChanged = 
      (this->fpV0 == pExistingVtx) ? &this->fpV0 :
      ((this->fpV1 == pExistingVtx) ? &this->fpV1 : NULL);
    assert(ppVtxToBeChanged != NULL);
    
    if (*ppVtxToBeChanged != NULL)
    {
      Adjacency::Unregister(*ppVtxToBeChanged, this);
    }
    
    if (pNewVtx != NULL)
    {
      Adjacency::Register(pNewVtx, this);
    }
    
    *ppVtxToBeChanged = pNewVtx;
  }
  
  Triangle* GetT0() const
  {
    if (AdjacencyContainer<Triangle>::fElements.size() > 0)
    {
      return AdjacencyContainer<Triangle>::fElements.front();
    }
    else
    {
      return NULL;
    }
  }
  
  Triangle* GetT1() const
  {
    if (AdjacencyContainer<Triangle>::fElements.size() > 1)
    {
      return AdjacencyContainer<Triangle>::fElements.back();
    }
    else
    {
      return NULL;
    }
  }
  
  bool GetIsActive() const { return this->fIsActive; }
  
private:
  Edge() 
  : fpV0(NULL)
  , fpV1(NULL)
  , fAge(static_cast<Age>(0))
  , fIsActive(false)
  {}
  
  ~Edge() {}
  void Delete() { delete this; }
  
  void Init()
  {
    this->fpV0 = NULL;
    this->fpV1 = NULL;
    this->fAge = static_cast<Age>(0);
    AdjacencyContainer<Triangle>::fElements.clear();
    this->fIsActive = true;
  }
  
  void SetVertices(Vertex* pV0, Vertex* pV1)
  {
    if (this->fpV0 != NULL && this->fpV1 != NULL)
    {
      Adjacency::Unregister(this->fpV0, this);
      Adjacency::Unregister(this->fpV1, this);
    }
    
    if (pV0 != NULL && pV1 != NULL)
    {
      Adjacency::Register(pV0, this);
      Adjacency::Register(pV1, this);
    }
    
    this->fpV0 = pV0;
    this->fpV1 = pV1;
  }
  
  void SetIsActive(bool isActive) { this->fIsActive = isActive; }
  
  Vertex* fpV0;
  Vertex* fpV1;
  
  Age fAge;
  
  bool fIsActive;
};  // class Edge


#endif  // #ifndef SURFACE_RECONSTRUCTION__EDGE_H_
