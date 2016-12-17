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

#ifndef SURFACE_RECONSTRUCTION__TEXTURE_H_
#define SURFACE_RECONSTRUCTION__TEXTURE_H_


#include <map>
#include <string>

#ifndef Q_MOC_RUN
#include <boost/noncopyable.hpp>
#endif

#include <common/vec3.h>

#include "container.h"
#include "data_types.h"


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
class Texture : boost::noncopyable
{
  CONTAINER_ELEMENT<Texture>;
  
public:
  friend class Textures;

  unsigned int GetId() const { return this->fId; }
  const std::string& GetFileName() const { return this->fFileName; }
  const Position& GetCamPosition() const { return this->fCamPosition; }
  const Vector& GetCamDirection() const { return this->fCamDirection; }
  
  unsigned int GetWidth() const { return this->fWidth; }
  unsigned int GetHeight() const { return this->fHeight; }
  
  static Texture* GetTexture(unsigned int id)
  {
    return Texture::sIdTextureMap[id];
  }
  
  Flt GetU(const Vector& point3d) const;
  Flt GetV(const Vector& point3d) const;
  
private:
  Texture(const std::string& fileName,
          const Position& camPosition,
          const Vector& camDirection,
          Flt m11, Flt m12, Flt m13,
          Flt m21, Flt m22, Flt m23,
          Flt m31, Flt m32, Flt m33,
          const Vector& offset,
          Flt offsetU, Flt offsetV)
  : fId(Texture::GetNextId())
  , fWidth(0u)
  , fHeight(0u)
  , fFileName(fileName)
  , fCamPosition(camPosition)
  , fCamDirection(camDirection)
  , fM11(m11)
  , fM12(m12)
  , fM13(m13)
  , fM21(m21)
  , fM22(m22)
  , fM23(m23)
  , fM31(m31)
  , fM32(m32)
  , fM33(m33)
  , fOffset(offset)
  , fOffsetU(offsetU)
  , fOffsetV(offsetV)
  {
    Texture::sIdTextureMap[this->fId] = this;
  }
  
  Texture(const unsigned int texId,
          const unsigned int width,
          const unsigned int height,
          const std::string& fileName,
          const Position& camPosition,
          const Vector& camDirection,
          Flt m11, Flt m12, Flt m13,
          Flt m21, Flt m22, Flt m23,
          Flt m31, Flt m32, Flt m33,
          const Vector& offset,
          Flt offsetU, Flt offsetV)
  : fId(texId)
  , fWidth(width)
  , fHeight(height)
  , fFileName(fileName)
  , fCamPosition(camPosition)
  , fCamDirection(camDirection)
  , fM11(m11)
  , fM12(m12)
  , fM13(m13)
  , fM21(m21)
  , fM22(m22)
  , fM23(m23)
  , fM31(m31)
  , fM32(m32)
  , fM33(m33)
  , fOffset(offset)
  , fOffsetU(offsetU)
  , fOffsetV(offsetV)
  {
    Texture::sNextId = this->fId + 1u;
    Texture::sIdTextureMap[this->fId] = this;
  }
  
  void Delete() { delete this; }
  
  unsigned int fId;
  unsigned int fWidth;
  unsigned int fHeight;
  std::string fFileName;
  Position fCamPosition;
  Vector fCamDirection;
  Flt fM11;
  Flt fM12;
  Flt fM13;
  Flt fM21;
  Flt fM22;
  Flt fM23;
  Flt fM31;
  Flt fM32;
  Flt fM33;
  Vector fOffset;
  Flt fOffsetU;
  Flt fOffsetV;
  
  static unsigned int GetNextId() { return Texture::sNextId++; }
  static unsigned int sNextId;
  static std::map<unsigned int, Texture*> sIdTextureMap;
};


#endif  // #ifndef SURFACE_RECONSTRUCTION__TEXTURE_H_
