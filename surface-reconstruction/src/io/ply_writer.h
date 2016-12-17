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

#ifndef SURFACE_RECONSTRUCTION__IO__PLY_WRITER_H_
#define SURFACE_RECONSTRUCTION__IO__PLY_WRITER_H_


#include <map>
#include <fstream>
#include <string>

#include <common/vec3_forward.h>

#include "data_types.h"
#include "vertex.h"


namespace io
{

class PlyWriter
{
public:
  /// \brief Constructor
  PlyWriter(const std::string& filename,
            unsigned int numOfVertices,
            unsigned int numOfFaces);
  
  
  
  ~PlyWriter()
  {
    this->fPlyFile.close();
  }
  
  
  
  void AddVertex(const Position& position,
#ifndef IGNORE_COLOUR 
    Flt r, Flt g, Flt b,
#endif    
     unsigned int id);
  void AddFace(unsigned int id0, unsigned int id1, unsigned int id2);
  
  
private:
  unsigned int fCurrentVertexID;
  std::map<unsigned int, unsigned int> fVertexIDs;
  std::ofstream fPlyFile;
};

} // namespace io

#endif  // #ifndef SURFACE_RECONSTRUCTION__IO__PLY_WRITER_H_
