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

#ifndef SURFACE_RECONSTRUCTION__INPUT_ADAPTER_H_
#define SURFACE_RECONSTRUCTION__INPUT_ADAPTER_H_


#include <string>

#include "data_types.h"
#include "io/input_adapter_interface.h"


// forward declarations
class Point;
class PointCloud;
class Textures;


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
class InputAdapter : public io::InputAdapterInterface<Flt>
{
public:
  InputAdapter(PointCloud* pPointCloud, Textures* pTextures);
  ~InputAdapter();
  
  void OnBeginPoint();
  void OnPointPosition(Flt x, Flt y, Flt z);
  void OnPointNormal(Flt x, Flt y, Flt z);
  void OnPointColour(Flt r, Flt g, Flt b);
  void OnPointTexCoord(unsigned int id, Flt u, Flt v);
  void OnEndPoint();
  
  void OnTexture(unsigned int id,
                 const std::string& fileName,
                 unsigned int width, unsigned int height,
                 Flt camPosX, Flt camPosY, Flt camPosZ,
                 Flt camDirX, Flt camDirY, Flt camDirZ,
                 Flt m11, Flt m12, Flt m13,
                 Flt m21, Flt m22, Flt m23,
                 Flt m31, Flt m32, Flt m33,
                 Flt offsetX, Flt offsetY, Flt offsetZ,
                 Flt offsetU, Flt offsetV);

private:
  PointCloud* fpPointCloud;
  Textures* fpTextures;
  
  Point* fpCurrentPoint;
};  // class


#endif  // #ifndef SURFACE_RECONSTRUCTION__INPUT_ADAPTER_H_
