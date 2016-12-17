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

#include "texture.h"

// init static variables
unsigned int Texture::sNextId = 0u;
std::map<unsigned int, Texture*> Texture::sIdTextureMap;





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Flt
Texture::GetU
(const Vector& point3d)
const
{
  return ((this->fM11 * point3d.GetX() +
           this->fM12 * point3d.GetY() +
           this->fM13 * point3d.GetZ() -
           this->fOffset.GetX()) /
          (this->fM31 * point3d.GetX() +
           this->fM32 * point3d.GetY() +
           this->fM33 * point3d.GetZ() -
           this->fOffset.GetZ()) +
          this->fOffsetU);
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Flt
Texture::GetV
(const Vector& point3d)
const
{
  return ((this->fM21 * point3d.GetX() +
           this->fM22 * point3d.GetY() +
           this->fM23 * point3d.GetZ() -
           this->fOffset.GetY()) /
          (this->fM31 * point3d.GetX() +
           this->fM32 * point3d.GetY() +
           this->fM33 * point3d.GetZ() -
           this->fOffset.GetZ()) +
          this->fOffsetV);
}
