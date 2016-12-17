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

#include <iostream>

#ifndef Q_MOC_RUN
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/range/algorithm/find_if.hpp>
#endif

#include <common/vec3.h>

#include "texture.h"
#include "texture_coordinate.h"


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TextureCoordinate::AdaptTo
(::Container<TextureCoordinate>::ConstRange texCoordsSource,
 Flt rate,
 Container<TextureCoordinate>::Range texCoordsDestination)
{
  typedef ::Container<TextureCoordinate> CTC;
  CTC::ConstIterator texCoordsSourceIter(texCoordsSource.begin());
  const CTC::ConstIterator texCoordsSourceEnd(texCoordsSource.end());
  CTC::Iterator texCoordsDestinationIter(texCoordsDestination.begin());
  CTC::ConstIterator texCoordsDestinationEnd(texCoordsDestination.end());
  
  while ((texCoordsSourceIter != texCoordsSourceEnd))
  {
    const unsigned int srcTexId = texCoordsSourceIter->GetId();

    // seek in destination
    while ((texCoordsDestinationIter->GetId() < srcTexId) &&
           (texCoordsDestinationIter != texCoordsDestinationEnd))
    {
      ++texCoordsDestinationIter;
    }
    
    // found in destination
    if ((texCoordsDestinationIter != texCoordsDestinationEnd) &&
        (texCoordsDestinationIter->GetId() == srcTexId))
    {
      // increase confidence
      texCoordsDestinationIter->fConfidence += sc<Flt>(1.0);
    }
    // did not find in destination
    else if ((texCoordsDestinationIter == texCoordsDestinationEnd) ||
             (texCoordsDestinationIter->GetId() > srcTexId))
    {
      TextureCoordinate::SortedAdd(
        *(new TextureCoordinate(texCoordsSourceIter->GetId(),
                                sc<Flt>(1.0))),
        texCoordsDestination);
    }
    
    ++texCoordsSourceIter;
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TextureCoordinate::SetAverage
(Container<TextureCoordinate>::ConstRange texCoordsA,
 Container<TextureCoordinate>::ConstRange texCoordsB,
 Container<TextureCoordinate>::Range newTexCoords)
{
  typedef ::Container<TextureCoordinate> CTC;

  CTC::ConstIterator texCoordsAIter(texCoordsA.begin());
  const CTC::ConstIterator texCoordsAEnd(texCoordsA.end());
  
  CTC::ConstIterator texCoordsBIter(texCoordsB.begin());
  const CTC::ConstIterator texCoordsBEnd(texCoordsB.end());
  
  while ((texCoordsAIter != texCoordsAEnd) &&
         (texCoordsBIter != texCoordsBEnd))
  {
    // seek in destination
    while ((texCoordsBIter->GetId() < texCoordsAIter->GetId()) &&
           (texCoordsBIter != texCoordsBEnd))
    {
      ++texCoordsBIter;
    }
    
    // found in destination
    if ((texCoordsBIter != texCoordsBEnd) &&
        (texCoordsBIter->GetId() == texCoordsAIter->GetId()))
    {
      const Flt confidence =
        static_cast<Flt>(texCoordsAIter->fConfidence +
                           texCoordsBIter->fConfidence) / 2.0f;
      TextureCoordinate::SortedAdd(*(new TextureCoordinate(texCoordsAIter->GetId(), /*u, v,*/ confidence)),
                                   newTexCoords);
    }
    
    ++texCoordsAIter;
    ++texCoordsBIter;
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TextureCoordinate::Modify
(const TextureCoordinate &texCoord,
 ::Container<TextureCoordinate>::Range range)
{
}
