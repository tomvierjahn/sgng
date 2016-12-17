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

#ifndef SURFACE_RECONSTRUCTION__DATA_TYPES_H_
#define SURFACE_RECONSTRUCTION__DATA_TYPES_H_

#include <common/vec3_forward.h>

typedef float Flt;
typedef Vec3<Flt> Vector;
typedef Vector Position;

typedef unsigned int Activity;

namespace
{

template <typename TTarget, typename TSource>
inline TTarget sc(TSource v)
{
  return static_cast<TTarget>(v);
}
  

struct Mat4
{
  Mat4()
  {
    for (unsigned int row = 0; row < 4; ++row)
    {
      for (unsigned int col = 0; col < 4; ++col)
      {
        this->m[4 * col + row] = (row == col) ? sc<Flt>(1.0) : sc<Flt>(0.0);
      }
    }
  }
  
  Flt* Ptr() { return this->m; }
  const Flt* Ptr() const { return this->m; }
  
  Flt& operator()(unsigned int row, unsigned int col)
  {
    return this->m[4 * col + row];
  }

  Flt operator()(unsigned int row, unsigned int col) const
  {
    return this->m[4 * col + row];
  }

  Flt* operator()() { return &m[0]; }
  
  Mat4 ScaleRotate()
  {
    Mat4 retVal;
    for (unsigned int row = 0; row < 4; ++row)
    {
      for (unsigned int col = 0; col < 3; ++col)
      {
        retVal(row, col) = (*this)(row, col);
      }
    }
    return retVal;
  }
  
  Mat4 Translate()
  {
    Mat4 retVal;
    for (unsigned int row = 0; row < 4; ++row)
    {
      retVal(row, 3) = (*this)(row, 3);
    }
    return retVal;
  }
  
  Flt m[16];
};
  
} // namespace
#endif  // #ifndef SURFACE_RECONSTRUCTION__DATA_TYPES_H_
