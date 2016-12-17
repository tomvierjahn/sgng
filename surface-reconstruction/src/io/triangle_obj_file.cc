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

#include <map>
#include <fstream>

#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#endif

#include "container.h"
#include "data_types.h"
#include "triangle.h"
#include "triangle_mesh.h"

#include "io/ply_reader.h"
#include "io/ply_writer.h"
#include "io/triangle_obj_file.h"

namespace io
{

////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleObjFile::Load
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
TriangleObjFile::Save(const TriangleMesh* pTriangleMesh)
const
{
  std::map<const Vertex*, unsigned int> vertexIDs;
  
  std::ofstream objFile[7];
  
  objFile[0].open("/Users/tom/Desktop/mesh1.obj");
  objFile[1].open("/Users/tom/Desktop/mesh2.obj");
  objFile[2].open("/Users/tom/Desktop/mesh3.obj");
  objFile[3].open("/Users/tom/Desktop/mesh4.obj");
  objFile[4].open("/Users/tom/Desktop/mesh5.obj");
  objFile[5].open("/Users/tom/Desktop/mesh6.obj");
  objFile[6].open("/Users/tom/Desktop/mesh7.obj");
  
  unsigned int id = 1u;
  
  if (objFile[0].is_open() && objFile[1].is_open() && objFile[2].is_open() &&
      objFile[3].is_open() && objFile[4].is_open() && objFile[5].is_open() &&
      objFile[6].is_open())
  {
    BOOST_FOREACH(const Vertex& vertex, pTriangleMesh->GetVertices())
    {
      for (unsigned int i = 0; i < 7; ++i)
      {
        objFile[i] << "v ";
        objFile[i] << vertex.GetPosition().GetX() << " ";
        objFile[i] << vertex.GetPosition().GetY() << " ";
        objFile[i] << vertex.GetPosition().GetZ() << std::endl;
      }
      vertexIDs[&vertex] = id++;
    }
    

    
    BOOST_FOREACH(const Triangle& tri, pTriangleMesh->GetTriangles())
    {
      unsigned int textureA = 0u;
      
      const Position p0 = tri.GetV0()->GetPosition();
      const Position p1 = tri.GetV1()->GetPosition();
      const Position p2 = tri.GetV2()->GetPosition();
      
      const Vector normal(
                          normalize(crossProduct(normalize(p1 - p0), normalize(p2 - p0))));
      const Vector barycenter((sc<Flt>(1.0) / sc<Flt>(3.0)) * p0 +
                              (sc<Flt>(1.0) / sc<Flt>(3.0)) * p1 +
                              (sc<Flt>(1.0) / sc<Flt>(3.0)) * p2);
      
#ifndef NO_TEXTURES
      Flt maxDotP = -1.0f;
      typedef ::Container<TextureCoordinate> CTC;
      CTC::ConstIterator texCoordsV0Iter(
        tri.GetV0()->GetTextureCoordinates().begin());
      CTC::ConstIterator texCoordsV1Iter(
        tri.GetV1()->GetTextureCoordinates().begin());
      CTC::ConstIterator texCoordsV2Iter(
        tri.GetV2()->GetTextureCoordinates().begin());
      for (unsigned int i = 0; i < 7; ++i)
      {
//        if(texCoordsV0Iter->IsValid() &&
//           texCoordsV1Iter->IsValid() &&
//           texCoordsV2Iter->IsValid())
        {
          //const Flt dotP = fabs(dotProduct(GLViewer::Cam1(), normal));
          const Flt dotPA = fabs(dotProduct(
                              normalize(texCoordsV0Iter->GetCamPosition()-barycenter),
                              normal));
          const Flt dotPB = fabs(dotProduct(
                              normalize(texCoordsV0Iter->GetCamPosition()-barycenter),
                              texCoordsV0Iter->GetCamDirection()));
          if (texCoordsV0Iter->GetConfidence() *
              texCoordsV1Iter->GetConfidence() *
              texCoordsV2Iter->GetConfidence() *
              dotPA * dotPB > maxDotP)
          {
            maxDotP =
              texCoordsV0Iter->GetConfidence() *
              texCoordsV1Iter->GetConfidence() *
              texCoordsV2Iter->GetConfidence() *
              dotPA * dotPB;
            textureA = i;
          }
        }
        ++texCoordsV0Iter;
        ++texCoordsV1Iter;
        ++texCoordsV2Iter;
      }
#endif
     
      objFile[textureA] << "f ";
      objFile[textureA] << vertexIDs[tri.GetV0()] << "/";
      objFile[textureA] << vertexIDs[tri.GetV0()] << " ";
      objFile[textureA] << vertexIDs[tri.GetV1()] << "/";
      objFile[textureA] << vertexIDs[tri.GetV1()] << " ";
      objFile[textureA] << vertexIDs[tri.GetV2()] << "/";
      objFile[textureA] << vertexIDs[tri.GetV2()] << std::endl;
    }
  }
  
  for (unsigned int i = 0; i < 7; ++i)
  {
    if (objFile[i].is_open())
    {
      objFile[i].close();
    }
  }
}




////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
TriangleObjFile::Delete
(const TriangleMesh*)
const
{
  assert(false);
}

} // namespace io
