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

#ifndef SURFACE_RECONSTRUCTION__POINT_H_
#define SURFACE_RECONSTRUCTION__POINT_H_


#include <vector>

#ifndef Q_MOC_RUN
#include <boost/noncopyable.hpp>
#include <no_warning/boost__range__iterator_range.h>
#endif

#include <common/vec3.h>

#include "container.h"
#include "data_types.h"
#include "program_options.h"
#include "texture_coordinate.h"


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
class Point : boost::noncopyable
{
public:
  friend class PointCloud;
  friend class TextureTest;
  typedef unsigned long ID;
#ifndef IGNORE_COLOUR
  typedef float ColorComponent;
#endif
  
  const Position& GetPosition() const { return this->fPosition; }
  const Vector& GetNormal() const { return this->fNormal; }
  
  void SetNormal(const Vector& n) { this->fNormal = n; }
  
#ifndef IGNORE_COLOUR
  ColorComponent GetRed() const { return this->fRed; }
  ColorComponent GetGreen() const { return this->fGreen; }
  ColorComponent GetBlue() const { return this->fBlue; }
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
  
private:
  Point(const Position& position
        , const Vector& normal = Vector(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0))
#ifndef IGNORE_COLOUR
        , ColorComponent red = sc<Flt>(1.0)
        , ColorComponent green = sc<Flt>(1.0)
        , ColorComponent blue = sc<Flt>(1.0)
#endif
        , Flt u1 = sc<Flt>(-1.0), Flt v1 = sc<Flt>(-1.0)
        , Flt u2 = sc<Flt>(-1.0), Flt v2 = sc<Flt>(-1.0)
        , Flt u3 = sc<Flt>(-1.0), Flt v3 = sc<Flt>(-1.0)
        , Flt u4 = sc<Flt>(-1.0), Flt v4 = sc<Flt>(-1.0)
        , Flt u5 = sc<Flt>(-1.0), Flt v5 = sc<Flt>(-1.0)
        , Flt u6 = sc<Flt>(-1.0), Flt v6 = sc<Flt>(-1.0)
        , Flt u7 = sc<Flt>(-1.0), Flt v7 = sc<Flt>(-1.0)
        )
  : fPosition(position)
  , fNormal(normal)
#ifndef IGNORE_COLOUR
  , fRed(red)
  , fGreen(green)
  , fBlue(blue)
#endif
  {
  }
  
  Point(Flt x, Flt y, Flt z)
  : fPosition(x, y, z)
  {
  }
  
#ifndef IGNORE_COLOUR
  void SetColour(Flt r, Flt g, Flt b)
  {
    this->fRed = r;
    this->fGreen = g;
    this->fBlue = b;
  }
#endif
  
  Position fPosition;
  Vector fNormal;
  
#ifndef IGNORE_COLOUR
  ColorComponent fRed;
  ColorComponent fGreen;
  ColorComponent fBlue;
#endif
  
#ifndef NO_TEXTURES
  ::Container<TextureCoordinate>::Type fTextureCoordinates;
#endif
  
  // used for container
public:
  typedef std::vector<Point*> Container;
  typedef boost::iterator_range<Container::iterator> Range;
  typedef boost::iterator_range<Container::const_iterator> ConstRange;

};  // class Point


#endif  // #ifndef SURFACE_RECONSTRUCTION__POINT_H_
