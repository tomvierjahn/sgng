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

#ifndef SURFACE_RECONSTRUCTION__IO__TRIANGLE_OBJ_FILE_H_
#define SURFACE_RECONSTRUCTION__IO__TRIANGLE_OBJ_FILE_H_


#include <string>

#include "io/data_storage.h"


class TriangleMesh;

namespace io
{
  
  class TriangleObjFile : public DataStorage<TriangleMesh>
  {
  public:
    /// \brief Constructor
    TriangleObjFile(const std::string& filename)
    : fFilename(filename)
    {
      this->SetInfo(filename);
    }
    
    virtual void Load(TriangleMesh* pTriangleMesh) const;
    
    virtual void Save(const TriangleMesh* pTriangleMesh) const;
    
    virtual void Delete(const TriangleMesh* pTriangleMesh) const;
    
  private:
    std::string fFilename;
  };
  
} // namespace io

#endif  // #ifndef SURFACE_RECONSTRUCTION__IO__TRIANGLE_OBJ_FILE_H_
