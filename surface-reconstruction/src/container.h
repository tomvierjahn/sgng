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

#ifndef SURFACE_RECONSTRUCTION__CONTAINER_H_
#define SURFACE_RECONSTRUCTION__CONTAINER_H_

#ifndef Q_MOC_RUN
#include <boost/intrusive/list.hpp>
#include <no_warning/boost__range__iterator_range.h>
#endif


typedef boost::intrusive::list_member_hook<> ContainerHook;

#define CONTAINER_ELEMENT private: ContainerHook fHook; friend struct ::Container 
#define ADD_CONTAINER_ELEMENT private: ContainerHook fAddHook; friend struct ::AddContainer


template <typename ElementType>
struct Container
{
  typedef boost::intrusive::list<
    ElementType, 
    boost::intrusive::member_hook<
      ElementType,
      ContainerHook,
      &ElementType::fHook>, 
    boost::intrusive::constant_time_size<true> > Type;
  typedef boost::iterator_range<typename Type::iterator> Range;
  typedef boost::iterator_range<typename Type::const_iterator> ConstRange;
  typedef typename Type::iterator Iterator;
  typedef typename Type::const_iterator ConstIterator;
  typedef typename Type::size_type SizeType;
};

template <typename ElementType>
struct AddContainer
{
  typedef boost::intrusive::list<
  ElementType,
  boost::intrusive::member_hook<
  ElementType,
  ContainerHook,
  &ElementType::fAddHook>,
  boost::intrusive::constant_time_size<true> > Type;
  typedef boost::iterator_range<typename Type::iterator> Range;
  typedef boost::iterator_range<typename Type::const_iterator> ConstRange;
  typedef typename Type::iterator Iterator;
  typedef typename Type::const_iterator ConstIterator;
  typedef typename Type::size_type SizeType;
};

#endif  // #ifndef SURFACE_RECONSTRUCTION__CONTAINER_H_
