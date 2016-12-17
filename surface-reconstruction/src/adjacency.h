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

#ifndef SURFACE_RECONSTRUCTION__ADJACENCY_H_
#define SURFACE_RECONSTRUCTION__ADJACENCY_H_


#include <list>

#include <no_warning/boost__range__iterator_range.h>


////////////////////////////////////////////////////////////////////////////////
/// \brief Base class implementing adjacency
///
/// Inheriting from this class adds, e.g., a list of adjacent edges to a vertex.
/// Iterators can be acquired via the static methods of struct Adjacent.
/// Adjacency can be modified via the struct methods of struct Adjacency.
///
/// \tparam AdjacentElement Type of the adjacent elements.
///                         Although we use pointers to store and access the
///                         elements, this must not be a pointer type
////////////////////////////////////////////////////////////////////////////////
template <typename AdjacentElement>
class AdjacencyContainer
{
public:
  // convenience
  typedef std::list<AdjacentElement*> Container;
  typedef typename Container::iterator Iterator;
  typedef typename Container::const_iterator ConstIterator;
  typedef typename boost::iterator_range<Iterator> Range;
  typedef typename boost::iterator_range<ConstIterator> ConstRange;
  
  template <typename T>
  friend struct Adjacent;
  friend struct Adjacency;
  
protected:
  Container fElements;  ///< \brief Stores the adjacent elements
  
private:
  /// \returns Iterator to the beginning of the list of elements (INTERNAL)
  /// \note To be used by struct Adjacent only
  Iterator Begin() { return this->fElements.begin(); }
  
  /// \returns ConstIterator to the beginning of the list of elements (INTERNAL)
  /// \note To be used by struct Adjacent only
  ConstIterator Begin() const { return this->fElements.begin(); }
  
  /// \returns Iterator past the end of the list of elements (INTERNAL)
  /// \note To be used by struct Adjacent only
  Iterator End() { return this->fElements.end(); }
  
  /// \returns ConstIterator past the end of the list of elements (INTERNAL)
  /// \note To be used by struct Adjacent only
  ConstIterator End() const { return this->fElements.end(); }
  
  /// \brief Register an adjacent element (INTERNAL)
  /// \note To be used by struct Adjacency only
  void Register(AdjacentElement* pAdjacentElement)
  {
    const ConstIterator found(std::find(this->fElements.begin(), 
                                        this->fElements.end(), 
                                        pAdjacentElement));
    if (found == this->fElements.end())
    {
      this->fElements.push_back(pAdjacentElement);
    }
  }
  
  /// \brief Unregister an adjacent element (INTERNAL)
  /// \note To be used by struct Adjacency only
  void Unregister(const AdjacentElement* pAdjacentElement)
  {
    Iterator found(std::find(this->fElements.begin(), 
                             this->fElements.end(), 
                             pAdjacentElement));
    if (found != this->fElements.end())
    {
      this->fElements.erase(found);
    }
  }
};  // class AdjacencyContainer





////////////////////////////////////////////////////////////////////////////////
/// \brief Struct to conveniently acquire iterators over adjacent elements
///
/// Convenient acces to iterator returning functions of AdjacencyContainer.
///
/// \tparam AdjacentElement Type of the adjacent elements.
///                         Although we use pointers to store and access the
///                         elements, this must not be a pointer type
////////////////////////////////////////////////////////////////////////////////
template <typename AdjacentElement>
struct Adjacent
{
  // convenience
  typedef AdjacencyContainer<AdjacentElement> AC;
  typedef typename AC::Iterator Iterator;
  typedef typename AC::ConstIterator ConstIterator;
  typedef typename AC::Range Range;
  typedef typename AC::ConstRange ConstRange;
  
  /// \returns  Iterator to the beginning of the list of elements
  /// \param    pElement  The element to iterate around
  template <typename T>
  static Iterator Begin(T* pElement) {return pElement->AC::Begin();}
  
  /// \returns  ConstIterator to the beginning of the list of elements
  /// \param    pElement  The element to iterate around
  template <typename T>
  static ConstIterator Begin(const T* pElement) {return pElement->AC::Begin();}
  
  /// \returns  Iterator past the end of the list of elements
  /// \param    pElement  The element to iterate around
  template <typename T>
  static Iterator End(T* pElement) {return pElement->AC::End();}
  
  /// \returns  ConstIterator past the end of the list of elements
  /// \param    pElement  The element to iterate around
  template <typename T>
  static ConstIterator End(const T* pElement) {return pElement->AC::End();}
  
  /// \returns  Range along the list of elements
  /// \param    pElement  The element to iterate around
  template <typename T>
  static Range Around(T* pElement) 
  { 
    return Range(pElement->AC::Begin(), pElement->AC::End()); 
  }
  
  /// \returns  ConstRange along the list of elements
  /// \param    pElement  The element to iterate around
  template <typename T>
  static ConstRange Around(const T* pElement) 
  { 
    return ConstRange(pElement->AC::Begin(), pElement->AC::End()); 
  }
};  // struct Adjacent





////////////////////////////////////////////////////////////////////////////////
/// \brief Struct to conveniently register and unregister Adjacency
///
/// Convenient acces to adjacency registering and unregistering methods
/// of AdjacencyContainer.
///
/// Example: Register edge \c pEdge as adjacent to vertex \c pVertex.
/// \code{.cpp}
/// Adjacency::Register(pVertex, pEdge);
/// \endcode
/// Adjacency::
////////////////////////////////////////////////////////////////////////////////
struct Adjacency
{
  /// \brief  Register pAdjacentElement as adjacent to pElement
  template <typename Element, typename AdjacentElement>
  static void Register(Element* pElement, AdjacentElement* pAdjacentElement)
  {
    pElement->AdjacencyContainer<AdjacentElement>::Register(pAdjacentElement);
  }
  
  /// \brief  Unregister pAdjacentElement as adjacent to pElement
  template <typename Element, typename AdjacentElement>
  static void Unregister(Element* pElement, 
                         const AdjacentElement* pAdjacentElement)
  {
    pElement->AdjacencyContainer<AdjacentElement>::Unregister(pAdjacentElement);
  }
};  // struct Adjacency


#endif  // #ifndef SURFACE_RECONSTRUCTION__ADJACENCY_H_
