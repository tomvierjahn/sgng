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

#ifndef SURFACE_RECONSTRUCTION__VERTEX_H_
#define SURFACE_RECONSTRUCTION__VERTEX_H_


#include <cassert>

#include <algorithm>
#include <vector>
#include <list>

#ifndef Q_MOC_RUN
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/noncopyable.hpp>
#endif

#include <common/vec3.h>

#include "adjacency.h"
#include "container.h"
#include "data_types.h"
#include "point.h"
#include "program_options.h"
#include "texture_coordinate.h"


class Edge;
class Triangle;
class OctreeNode;


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
class Vertex : boost::noncopyable, public AdjacencyContainer<Edge>
{
  CONTAINER_ELEMENT<Vertex>;
  ADD_CONTAINER_ELEMENT<Vertex>;

public:
  friend class TriangleMesh;
  
  static Activity ActivityIncrement() { return sc<Activity>(1u); }
  
  const Position& GetPosition() const { return this->fPosition; }
  
  Activity GetActivity() const { return this->fActivity; }
  
  unsigned int GetBmuCount() const { return this->fBmuCount; }
  
  unsigned int GetId() const { return this->fId; }
  
  bool IsDangling() const 
  { 
    return (AdjacencyContainer<Edge>::fElements.size() == 0); 
  }
  
  unsigned int GetValence() const 
  { 
    return AdjacencyContainer<Edge>::fElements.size(); 
  }
  
  void SetLastActiveIn(unsigned int iteration)
  { 
    this->fLastActiveIn = iteration;
  }
  unsigned int GetLastActiveIn() const { return this->fLastActiveIn; }
  
  void SetParentOctreeNode(OctreeNode* pParentOctreeNode)
  {
    this->fpParentOctreeNode = pParentOctreeNode;
  }
  
  OctreeNode* GetParentOctreeNode() const { return this->fpParentOctreeNode; }
  
#ifndef IGNORE_COLOUR
  Point::ColorComponent GetRed() const { return this->fRed; }
  Point::ColorComponent GetGreen() const { return this->fGreen; }
  Point::ColorComponent GetBlue() const { return this->fBlue; }
  
  void SetRed(Point::ColorComponent red) { this->fRed = red; }
  void SetGreen(Point::ColorComponent green) { this->fGreen = green; }
  void SetBlue(Point::ColorComponent blue) { this->fBlue = blue; }
#endif
  
#ifndef NO_TEXTURES
  ::Container<TextureCoordinate>::Range GetTextureCoordinates()
  {
    return this->fTextureCoordinates;
  }
  
  ::Container<TextureCoordinate>::ConstRange GetTextureCoordinates() const
  {
    return this->fTextureCoordinates;
  }
#endif
  
  void SetNormal(const Vector& normal) { this->fNormal = normalize(normal); }
  const Vector& GetNormal() const { return this->fNormal; }
  
  void IncAdaptCount() { this->fAdaptCount = sc<Flt>(0.99) * this->fAdaptCount + sc<Flt>(1.0); }
  void SetAdaptCount(Flt count) { this->fAdaptCount = count; }
  Flt AdaptCount() const { return this->fAdaptCount; }
  
  
  
  
private:
  Vertex()
  : fActivity(sc<Activity>(0))
  , fBmuCount(0u)
  , fLastActiveIn(0)
  , fpParentOctreeNode(NULL)
#ifndef IGNORE_COLOUR
  , fRed(static_cast<Point::ColorComponent>(1.0))
  , fGreen(static_cast<Point::ColorComponent>(1.0))
  , fBlue(static_cast<Point::ColorComponent>(1.0))
#endif
  , fInActiveList(false)
  , fNormal(Vector(0.0, 1.0, 0.0))
  , fAdaptCount(sc<Flt>(0.0))
  {
  }
  
  ~Vertex() {}
  void Delete() { delete this; }
  
  void Init()
  {
    static unsigned int lastId = 0;
    
    this->fActivity = sc<Activity>(0);
    this->fBmuCount = 0u;
    this->fId = lastId;
    ++lastId;
    AdjacencyContainer<Edge>::fElements.clear();
    this->fLastActiveIn = 0;
    this->fInActiveList = false;
#ifndef NO_TEXTURES
    this->fTextureCoordinates.clear_and_dispose(boost::bind(&TextureCoordinate::Delete, _1));
#endif
    this->fNormal = Vector(0.0, 1.0, 0.0);
    this->fAdaptCount = sc<Flt>(0.0);
    
//    typedef ::Container<TextureCoordinate> CTC;
//    CTC::Iterator texCoordIter(this->GetTextureCoordinates().begin());
//    for (unsigned int i = 0; i < 100; ++i)
//    {
//      texCoordIter->Init();
//      ++texCoordIter;
//    }
  }
  
  void SetPosition(const Position& position)
  {
    this->fPosition = position;
  }
  
  void SetActivity(Activity activity) { this->fActivity = activity; }
  void IncrementActivity() { this->fActivity += Vertex::ActivityIncrement(); }
  
  void SetBmuCount(unsigned int bmuCount) { this->fBmuCount = bmuCount; }
  void IncrementBmuCount() { ++(this->fBmuCount); }
  
  bool IsInActiveList() const { return this->fInActiveList; }
  void SetInActiveList(bool flag) { this->fInActiveList = flag; }
  
  
  Position fPosition;
  
  Activity fActivity;
  unsigned int fBmuCount;
  
  unsigned int fId;
  
  unsigned int fLastActiveIn;
  
  OctreeNode* fpParentOctreeNode;
  
#ifndef IGNORE_COLOUR
  Point::ColorComponent fRed;
  Point::ColorComponent fGreen;
  Point::ColorComponent fBlue;
#endif
  
#ifndef NO_TEXTURES
  ::Container<TextureCoordinate>::Type fTextureCoordinates;
#endif
  
  bool fInActiveList;
  
  
  Vector fNormal;
  
  Flt fAdaptCount;
  
};  // class Vertex

#endif  // #ifndef SURFACE_RECONSTRUCTION__VERTEX_H_
