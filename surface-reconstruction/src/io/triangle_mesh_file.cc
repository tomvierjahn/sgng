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
#include <boost/foreach.hpp>
#endif

#include "data_types.h"
#include "triangle.h"
#include "triangle_mesh.h"

#include "io/ply_reader.h"
#include "io/ply_writer.h"
#include "io/triangle_mesh_file.h"




namespace io
{

////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMeshFile::Load
(TriangleMesh* pTriangleMesh)
const
{
  // this needs implementation
  assert(false);
}




////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMeshFile::Save(const TriangleMesh* pTriangleMesh)
const
{
  PlyWriter writer(this->fFilename,
                   pTriangleMesh->GetNumOfVertices(),
                   pTriangleMesh->GetNumOfTriangles());
  
  BOOST_FOREACH(const Vertex& vertex, pTriangleMesh->GetVertices())
  {
    writer.AddVertex(vertex.GetPosition(),
    #ifndef IGNORE_COLOUR
     vertex.GetRed(), vertex.GetGreen(), vertex.GetBlue(),
    #endif
    vertex.GetId());
  }
  
  BOOST_FOREACH(const Triangle& triangle, pTriangleMesh->GetTriangles())
  {
    const Vertex* pA = triangle.GetV0();
    const Vertex* pB = triangle.GetV1();
    const Vertex* pC = triangle.GetV2();
    writer.AddFace(pA->GetId(), pB->GetId(), pC->GetId());
  }
}




////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleMeshFile::Delete
(const TriangleMesh*)
const
{
  PlyWriter writer(this->fFilename, 0, 0);
}

} // namespace io
