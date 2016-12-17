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

#ifndef Q_MOC_RUN
#include <boost/bind.hpp>
#endif

#include "textures.h"


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Textures::Textures
()
: fDirty(true)
{
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Textures::~Textures
()
{
  this->Clear();
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Texture*
Textures::NewTexture
(const std::string& fileName,
 const Position& camPosition,
 const Vector& camDirection,
 Flt m11, Flt m12, Flt m13,
 Flt m21, Flt m22, Flt m23,
 Flt m31, Flt m32, Flt m33,
 const Vector& offset,
 Flt offsetU, Flt offsetV)
{
  Texture* const pT = new Texture(fileName, camPosition, camDirection,
                                  m11, m12, m13,
                                  m21, m22, m23,
                                  m31, m32, m33,
                                  offset,
                                  offsetU, offsetV);
  this->fTextures.push_back(*pT);
  this->fDirty = true;
  return pT;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Texture*
Textures::NewTexture
(unsigned int texId,
 unsigned int width,
 unsigned int height,
 const std::string& fileName,
 const Position& camPosition,
 const Vector& camDirection,
 Flt m11, Flt m12, Flt m13,
 Flt m21, Flt m22, Flt m23,
 Flt m31, Flt m32, Flt m33,
 const Vector& offset,
 Flt offsetU, Flt offsetV)
{
  Texture* const pT = new Texture(texId, width, height,
                                  fileName, camPosition, camDirection,
                                  m11, m12, m13,
                                  m21, m22, m23,
                                  m31, m32, m33,
                                  offset,
                                  offsetU, offsetV);
  this->fTextures.push_back(*pT);
  this->fDirty = true;
  return pT;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
Textures::Clear
()
{
  this->fTextures.clear_and_dispose(boost::bind(&Texture::Delete, _1));
  this->fDirty = true;
}
