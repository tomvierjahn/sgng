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

#include <common/vec3.h>

#include "data_types.h"
#include "io/ply_writer.h"



namespace io
{

PlyWriter::PlyWriter
(const std::string& filename,
 unsigned int numOfVertices,
 unsigned int numOfFaces)
: fCurrentVertexID(0)
, fVertexIDs()
, fPlyFile(filename.c_str())
{
  if (this->fPlyFile.is_open())
  {
    this->fPlyFile << "ply" << std::endl;
    this->fPlyFile << "format ascii 1.0" << std::endl;
    this->fPlyFile << "element vertex " << numOfVertices << std::endl;
    this->fPlyFile << "property float x" << std::endl;
    this->fPlyFile << "property float y" << std::endl;
    this->fPlyFile << "property float z" << std::endl;
#ifndef IGNORE_COLOUR
    this->fPlyFile << "property float red" << std::endl;
    this->fPlyFile << "property float green" << std::endl;
    this->fPlyFile << "property float blue" << std::endl;
#endif
    this->fPlyFile << "element face " << numOfFaces << std::endl;
    this->fPlyFile << "property list uchar int vertex_indices" << std::endl;
    this->fPlyFile << "end_header" << std::endl;
  }
}


void
PlyWriter::AddVertex
(const Position& position,
#ifndef IGNORE_COLOUR
   Flt r, Flt g, Flt b, 
#endif   
   unsigned int id)
{
  this->fVertexIDs[id] = this->fCurrentVertexID;
  
  if (this->fPlyFile.is_open())
  {
    this->fPlyFile << position.GetX() << " ";
    this->fPlyFile << position.GetY() << " ";
    this->fPlyFile << position.GetZ() ;
#ifndef IGNORE_COLOUR
    this->fPlyFile << " " << r << " " << g << " " << b;
#endif
    this->fPlyFile << std::endl;
  }
  
  this->fCurrentVertexID++;
}





void
PlyWriter::AddFace
(unsigned int id0, unsigned int id1, unsigned int id2)
{
  if (this->fPlyFile.is_open())
  {
    this->fPlyFile << "3 ";
    this->fPlyFile << this->fVertexIDs[id0] << " ";
    this->fPlyFile << this->fVertexIDs[id1] << " ";
    this->fPlyFile << this->fVertexIDs[id2] << std::endl;
  }
}

} // namespace io
