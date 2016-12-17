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

#include <cstdlib>

#include <common/logging.h>

#include "input_adapter.h"
#include "point.h"
#include "point_cloud.h"
#include "texture_coordinate.h"
#include "textures.h"



////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
InputAdapter::InputAdapter
(PointCloud* pPointCloud,
 Textures* pTextures)
: fpPointCloud(pPointCloud)
, fpTextures(pTextures)
, fpCurrentPoint(NULL)
{
  if (this->fpPointCloud == NULL)
  {
    LOG_ERROR("Error processing input.");
    LOG_ERROR("Point cloud must not be NULL for this adapter.");
    LOG_ERROR("Terminating.");
    exit(EXIT_FAILURE);
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
InputAdapter::~InputAdapter
()
{
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
InputAdapter::OnBeginPoint
()
{
  if (this->fpCurrentPoint != NULL)
  {
    LOG_ERROR("Error processing input.");
    LOG_ERROR("Input adapter did not finish processing previous point.");
    LOG_ERROR("Terminating.");
    exit(EXIT_FAILURE);
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
InputAdapter::OnPointPosition
(Flt x, Flt y, Flt z)
{
  if (this->fpCurrentPoint != NULL)
  {
    LOG_ERROR("Error processing input.");
    LOG_ERROR("Input adapter did not finish processing previous point.");
    LOG_ERROR("Terminating.");
    exit(EXIT_FAILURE);
  }
  
  this->fpCurrentPoint = this->fpPointCloud->NewPoint(Position(x, y, z));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
InputAdapter::OnPointNormal
(Flt x, Flt y, Flt z)
{
  if (this->fpCurrentPoint == NULL)
  {
    LOG_ERROR("Error processing input.");
    LOG_ERROR("Input adapter did not create a point for normal.");
    LOG_ERROR("Terminating.");
    exit(EXIT_FAILURE);
  }
  
  this->fpCurrentPoint->SetNormal(Vector(x, y, z));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
InputAdapter::OnPointColour
(Flt r, Flt g, Flt b)
{
#ifndef IGNORE_COLOUR
  if (this->fpCurrentPoint != NULL)
  {
    this->fpPointCloud->SetPointColour(r, g, b, fpCurrentPoint);
  }
#endif
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
InputAdapter::OnPointTexCoord
(unsigned int id, Flt u, Flt v)
{
#ifndef NO_TEXTURES
  if (this->fpCurrentPoint != NULL)
  {
    if ((u >= 0.0f) && (v >= 0.0f))
    {
      TextureCoordinate::SortedAdd(
        *(new TextureCoordinate(id/*, u, v*/)),
        this->fpCurrentPoint->GetTextureCoordinates());
    }
  }
#endif
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
InputAdapter::OnEndPoint
()
{
  this->fpCurrentPoint = NULL;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
InputAdapter::OnTexture
(unsigned int id,
 const std::string &fileName,
 unsigned int width,
 unsigned int height,
 Flt camPosX,
 Flt camPosY,
 Flt camPosZ,
 Flt camDirX,
 Flt camDirY,
 Flt camDirZ,
 Flt m11, Flt m12, Flt m13,
 Flt m21, Flt m22, Flt m23,
 Flt m31, Flt m32, Flt m33,
 Flt offsetX, Flt offsetY, Flt offsetZ,
 Flt offsetU, Flt offsetV)
{
  if (this->fpTextures != NULL)
  {
    this->fpTextures->NewTexture(id,
                                 width, height,
                                 fileName,
                                 Position(camPosX, camPosY, camPosZ),
                                 Vector(camDirX, camDirY, camDirZ),
                                 m11, m12, m13,
                                 m21, m22, m23,
                                 m31, m32, m33,
                                 Vector(offsetX, offsetY, offsetZ),
                                 offsetU, offsetV);
  }
}
