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

#ifndef SURFACE_RECONSTRUCTION__IO__DATA_STORAGE_H_
#define SURFACE_RECONSTRUCTION__IO__DATA_STORAGE_H_

#include <string>

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#endif


namespace io
{

template <typename T>
class DataStorage
{
public:
  typedef boost::shared_ptr<DataStorage> SharedPtr;
  
  DataStorage() : fInfo("none") {}
  virtual ~DataStorage() {}
  
  virtual void Load(T*) const = 0;
  virtual void Save(const T*) const = 0;
  virtual void Delete(const T*) const = 0;
  
  void SetInfo(const std::string& info)
  {
    if (info.size() > 40)
    {
      this->fInfo = info.substr(0, 18);
      this->fInfo += "...";
      this->fInfo += info.substr(info.size() - 19, 19); 
    }
    else
    {
      this->fInfo = info;
    }
  }
  
  virtual std::string GetInfo() const { return this->fInfo; }
  virtual std::string GetFileName() const { return std::string(""); }
  
private:
  std::string fInfo;
};

} // namespace io

#endif  // #ifndef SURFACE_RECONSTRUCTION__IO__DATA_STORAGE_H_
