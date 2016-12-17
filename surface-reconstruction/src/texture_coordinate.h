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

#ifndef SURFACE_RECONSTRUCTION__TEXTURE_COORDINATE_H_
#define SURFACE_RECONSTRUCTION__TEXTURE_COORDINATE_H_

#ifndef Q_MOC_RUN
#include <boost/noncopyable.hpp>
#endif

#include <common/vec3_forward.h>

#include "container.h"
#include "data_types.h"
#include "texture.h"

////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
class TextureCoordinate : boost::noncopyable
{
  CONTAINER_ELEMENT<TextureCoordinate>;
  
public:
  TextureCoordinate(unsigned int textureId)
  : fTextureId(textureId)
//  , fU(sc<Flt>(-1.0))
//  , fV(sc<Flt>(-1.0))
  , fConfidence(sc<Flt>(0.0))
  {}
  
//  explicit TextureCoordinate(unsigned int textureId, Flt u, Flt v)
//  : fTextureId(textureId)
////  , fU(u)
////  , fV(v)
//  , fConfidence(sc<Flt>(0.0))
//  {}
  
  explicit TextureCoordinate(unsigned int textureId, //Flt u, Flt v,
                             int confidence)
  : fTextureId(textureId)
//  , fU(u)
//  , fV(v)
  , fConfidence(confidence)
  {}
  
  explicit TextureCoordinate(const TextureCoordinate& other)
  : fTextureId(other.fTextureId)
//  , fU(other.fU)
//  , fV(other.fV)
  , fConfidence(sc<Flt>(0.0))
  {}
  
  void Delete() { delete this; }
  
  void Init()
  {
//    this->fU = sc<Flt>(-1.0);
//    this->fV = sc<Flt>(-1.0);
    this->fConfidence = sc<Flt>(0.0);
  }
  
//  void Set(const TextureCoordinate& other)
//  {
//    this->fU = other.GetU();
//    this->fV = other.GetV();
//  }
//  void Set(Flt u, Flt v)
//  {
//    this->fU = u;
//    this->fV = v;
//  }
//  void SetU(Flt u) { this->fU = u; }
//  void SetV(Flt v) { this->fV = v; }
  void SetConfidence(Flt confidence) { this->fConfidence = confidence; }
  
//  Flt GetU() const { return this->fU; }
//  Flt GetV() const { return this->fV; }
  Flt GetConfidence() const { return this->fConfidence; }
  
//  bool IsValid() const
//  {
//    return ((this->fU >= sc<Flt>(0.0)) && (this->fV >= sc<Flt>(0.0)));
//  }
//  bool IsInvalid() const
//  {
//    return ((this->fU < sc<Flt>(0.0)) && (this->fV < sc<Flt>(0.0)));
//  }
  
  static void AdaptTo(::Container<TextureCoordinate>::ConstRange texCoordsSource,
                      Flt rate,
                      ::Container<TextureCoordinate>::Range texCoordsDestination);
  
  static void SetAverage(::Container<TextureCoordinate>::ConstRange texCoordsA,
                         ::Container<TextureCoordinate>::ConstRange texCoordsB,
                         ::Container<TextureCoordinate>::Range newTexCoords);
  
  const Position& GetCamPosition() const
  {
    return Texture::GetTexture(this->fTextureId)->GetCamPosition();
  }
  
  const Vector& GetCamDirection() const
  {
    return Texture::GetTexture(this->fTextureId)->GetCamDirection();
  }
  
  unsigned int GetId() const { return this->fTextureId; }
  
  bool operator==(const TextureCoordinate& other) const
  {
    return (this == &other);
  }
  
  static void SortedAdd(TextureCoordinate& coordinate,
                        ::Container<TextureCoordinate>::Type* pContainer)
  {
    ::Container<TextureCoordinate>::Iterator texCoordIter(pContainer->begin());
    const ::Container<TextureCoordinate>::ConstIterator texCoordEnd(pContainer->end());
    
    while ((texCoordIter != texCoordEnd) &&
           (texCoordIter->GetId() < coordinate.GetId()))
    {
      ++texCoordIter;
    }
    pContainer->insert(texCoordIter, coordinate);
  }
  
  static void SortedAdd(TextureCoordinate& coordinate,
                        ::Container<TextureCoordinate>::Range containerRange)
  {
    ::Container<TextureCoordinate>::Iterator texCoordIter(containerRange.begin());
    ::Container<TextureCoordinate>::ConstIterator texCoordEnd(containerRange.end());
    
    while ((texCoordIter != texCoordEnd) &&
           (texCoordIter->GetId() < coordinate.GetId()))
    {
      ++texCoordIter;
    }
    ::Container<TextureCoordinate>::Type::container_from_end_iterator(
      containerRange.end()).insert(texCoordIter, coordinate);
  }
  
  static void Modify(const TextureCoordinate& texCoord,
                     ::Container<TextureCoordinate>::Range range);
  
private:
  void AdaptTo(const TextureCoordinate& other, Flt rate)
  {
//    this->fU = this->fU + rate * (other.fU - this->fU);
//    this->fV = this->fV + rate * (other.fV - this->fV);
  }
  
  unsigned int fTextureId;
//  Flt fU;
//  Flt fV;
  Flt fConfidence;
};

#endif  // #ifndef SURFACE_RECONSTRUCTION__TEXTURE_COORDINATE_H_
