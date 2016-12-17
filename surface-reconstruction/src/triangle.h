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

#ifndef SURFACE_RECONSTRUCTION__TRIANGLE_H_
#define SURFACE_RECONSTRUCTION__TRIANGLE_H_


#ifndef Q_MOC_RUN
#include <boost/noncopyable.hpp>
#endif

#include "adjacency.h"
#include "container.h"
#include "edge.h"
#include "vertex.h"


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
class Triangle : boost::noncopyable
{
  CONTAINER_ELEMENT<Triangle>;

public:
  friend class TriangleMesh;
  
  Vertex* GetV0() { return this->fpV0; }
  Vertex* GetV1() { return this->fpV1; }
  Vertex* GetV2() { return this->fpV2; }

  const Vertex* GetV0() const { return this->fpV0; }
  const Vertex* GetV1() const { return this->fpV1; }
  const Vertex* GetV2() const { return this->fpV2; }
  
  Edge* GetE0() { return this->fpE0; }
  Edge* GetE1() { return this->fpE1; }
  Edge* GetE2() { return this->fpE2; }
  
  const Edge* GetE0() const { return this->fpE0; }
  const Edge* GetE1() const { return this->fpE1; }
  const Edge* GetE2() const { return this->fpE2; }
  
  Vertex* GetOther(const Vertex* pVA, const Vertex* pVB)
  const
  {
    if ((this->fpV0 != pVA) && (this->fpV0 != pVB))
    {
      return this->fpV0;
    }
    else if ((this->fpV1 != pVA) && (this->fpV1 != pVB))
    {
      return this->fpV1;
    }
    else if ((this->fpV2 != pVA) && (this->fpV2 != pVB))
    {
      return this->fpV2;
    }
    assert(false);
    return NULL;
  }
  
  void ReplaceEdge(Edge* pExistingEdge, Edge* pNewEdge)
  {
    // pExistingEdge must not necessarily exist in this triangle
    // it might have been replaced on another edge
    if (this->fpE0 == pExistingEdge)
    {
      this->SetEdges(pNewEdge, this->fpE1, this->fpE2);
    }
    if (this->fpE1 == pExistingEdge)
    {
      this->SetEdges(this->fpE0, pNewEdge, this->fpE2);
    }
    if (this->fpE2 == pExistingEdge)
    {
      this->SetEdges(this->fpE0, this->fpE1, pNewEdge);
    }
  }
  
  void DetermineVertices()
  {
    if ((this->fpE0 != NULL) && (this->fpE1 != NULL) && (this->fpE2 != NULL))
    {
      this->fpV0 = this->fpE0->GetV0();
      this->fpV1 = this->fpE0->GetV1();
      this->fpV2 = (((this->fpE1->GetV0() != this->fpV0) &&
                     (this->fpE1->GetV0() != this->fpV1)) ?
                    this->fpE1->GetV0() : this->fpE1->GetV1());
    }
    else
    {
      this->fpV0 = NULL;
      this->fpV1 = NULL;
      this->fpV2 = NULL;
    }
  }
  
  
  void SetUV0(Flt u, Flt v)
  {
    this->fU0 = u;
    this->fV0 = v;
  }
  void SetUV1(Flt u, Flt v)
  {
    this->fU1 = u;
    this->fV1 = v;
  }
  void SetUV2(Flt u, Flt v)
  {
    this->fU2 = u;
    this->fV2 = v;
  }
  
  Flt GetTexU0() const { return this->fU0; }
  Flt GetTexU1() const { return this->fU1; }
  Flt GetTexU2() const { return this->fU2; }
  Flt GetTexV0() const { return this->fV0; }
  Flt GetTexV1() const { return this->fV1; }
  Flt GetTexV2() const { return this->fV2; }
  
  void SetNormal(const Vector& normal)
  {
	  this->fNormal = normalize(normal);
  }

  Vector GetNormal() const { return this->fNormal; }
  
  bool NeedsReorient() const
  {
    const Vector edge0(this->fpV1->GetPosition() - this->fpV0->GetPosition());
    const Vector edge1(this->fpV2->GetPosition() - this->fpV0->GetPosition());
    const Vector normal(normalize(crossProduct(edge0, edge1)));
    
    const Vector vtxNormal(this->fpV0->GetNormal() +
                           this->fpV1->GetNormal() +
                           this->fpV2->GetNormal());
    const Flt dot = dotProduct(normal, vtxNormal);
    return dot < sc<Flt>(0.0);
  }
  
  unsigned int GetAge() const { return this->fAge; }
  void SetAge(unsigned int age) { this->fAge = age; }
  void IncAge() { ++this->fAge; }
  void DecAge()
  {
    if (this->fAge > 0)
    {
      --this->fAge;
    }
  }


  
private:
  Triangle()
  : fpV0(NULL)
  , fpV1(NULL)
  , fpV2(NULL)
  , fpE0(NULL)
  , fpE1(NULL)
  , fpE2(NULL)
  , fAge(0u)
  {
  }
  
  void Init()
  {
    this->fpV0 = NULL;
    this->fpV1 = NULL;
    this->fpV2 = NULL;
    
    this->fpE0 = NULL;
    this->fpE1 = NULL;
    this->fpE2 = NULL;
    
    this->fAge = 0u;
  }
  
  ~Triangle() {}
  void Delete() { delete this; }
  
  void SetEdges(Edge* pE0, Edge* pE1, Edge* pE2)
  {
    if ((this->fpE0 != NULL) && (this->fpE1 != NULL) && (this->fpE2 != NULL))
    {
      Adjacency::Unregister(this->fpE0, this);
      Adjacency::Unregister(this->fpE1, this);
      Adjacency::Unregister(this->fpE2, this);
    }
    
    if (pE0 != NULL && pE1 != NULL && pE2 != NULL)
    {
      Adjacency::Register(pE0, this);
      Adjacency::Register(pE1, this);
      Adjacency::Register(pE2, this);
    }
    
    this->fpE0 = pE0;
    this->fpE1 = pE1;
    this->fpE2 = pE2;
    
    this->DetermineVertices();
  }
  
  Vertex* fpV0;
  Vertex* fpV1;
  Vertex* fpV2;
  
  Edge* fpE0;
  Edge* fpE1;
  Edge* fpE2;
  
  
  Flt fU0;
  Flt fV0;
  Flt fU1;
  Flt fV1;
  Flt fU2;
  Flt fV2;
  
  Vector fNormal;
  unsigned int fAge;

};  // class Triangle


#endif  // #ifndef SURFACE_RECONSTRUCTION__TRIANGLE_H_
