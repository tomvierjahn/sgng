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

#include <common/unused.h>

#ifndef Q_MOC_RUN
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#endif

#include <gtest/gtest.h>

#include "data_types.h"
#include "point.h"
#include "texture_coordinate.h"

#ifndef NO_TEXTURES
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
class TextureTest : public ::testing::Test
{
public:
  static Point* NewPoint(Flt x, Flt y, Flt z) { return new Point(x, y, z); }
  static void PushBackTextureCoordinate(Point* pPoint, unsigned int id, Flt u, Flt v)
  {
    pPoint->fTextureCoordinates.push_back(*(new TextureCoordinate(id, u, v)));
  }
  static void SortedAddTextureCoordinate(Point* pPoint, unsigned int id, Flt u, Flt v)
  {
    TextureCoordinate::SortedAdd(*(new TextureCoordinate(id, u, v)),
                                 &(pPoint->fTextureCoordinates));
  }
  static void SortedAddTextureCoordinateByRange
  (Point* pPoint, unsigned int id, Flt u, Flt v)
  {
    TextureCoordinate::SortedAdd(*(new TextureCoordinate(id, u, v)),
                                 pPoint->GetTextureCoordinates());
  }
  
  static void RemoveAndDisposeTextureCoordinate(const TextureCoordinate& coord,
                                                Point* pPoint)
  {
    ::Container<TextureCoordinate>::Type::container_from_end_iterator(pPoint->GetTextureCoordinates().end()).remove_and_dispose(coord, boost::bind(&TextureCoordinate::Delete,_1));
  }
};


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
TEST_F(TextureTest, PushBackAdd)
{
  Point* pPoint = TextureTest::NewPoint(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0));
  
  TextureTest::PushBackTextureCoordinate(pPoint, 0u, sc<Flt>(0.1), sc<Flt>(0.01));
  TextureTest::PushBackTextureCoordinate(pPoint, 1u, sc<Flt>(0.2), sc<Flt>(0.02));
  TextureTest::PushBackTextureCoordinate(pPoint, 2u, sc<Flt>(0.3), sc<Flt>(0.03));
  TextureTest::PushBackTextureCoordinate(pPoint, 3u, sc<Flt>(0.4), sc<Flt>(0.04));
  TextureTest::PushBackTextureCoordinate(pPoint, 4u, sc<Flt>(0.5), sc<Flt>(0.05));
  TextureTest::PushBackTextureCoordinate(pPoint, 5u, sc<Flt>(0.6), sc<Flt>(0.06));
  TextureTest::PushBackTextureCoordinate(pPoint, 6u, sc<Flt>(0.7), sc<Flt>(0.07));
  
  ::Container<TextureCoordinate>::ConstIterator texCoordIter(pPoint->GetTextureCoordinates().begin());
  
  EXPECT_EQ(0u, (texCoordIter++)->GetId());
  EXPECT_EQ(1u, (texCoordIter++)->GetId());
  EXPECT_EQ(2u, (texCoordIter++)->GetId());
  EXPECT_EQ(3u, (texCoordIter++)->GetId());
  EXPECT_EQ(4u, (texCoordIter++)->GetId());
  EXPECT_EQ(5u, (texCoordIter++)->GetId());
  EXPECT_EQ(6u, (texCoordIter++)->GetId());
}


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
TEST_F(TextureTest, PushBackAddMissing)
{
  Point* pPoint = TextureTest::NewPoint(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0));
  
  TextureTest::PushBackTextureCoordinate(pPoint, 0u, sc<Flt>(0.1), sc<Flt>(0.01));
  TextureTest::PushBackTextureCoordinate(pPoint, 2u, sc<Flt>(0.3), sc<Flt>(0.03));
  TextureTest::PushBackTextureCoordinate(pPoint, 5u, sc<Flt>(0.6), sc<Flt>(0.06));
  TextureTest::PushBackTextureCoordinate(pPoint, 6u, sc<Flt>(0.7), sc<Flt>(0.07));
  
  ::Container<TextureCoordinate>::ConstIterator texCoordIter(pPoint->GetTextureCoordinates().begin());
  
  EXPECT_EQ(0u, (texCoordIter++)->GetId());
  EXPECT_EQ(2u, (texCoordIter++)->GetId());
  EXPECT_EQ(5u, (texCoordIter++)->GetId());
  EXPECT_EQ(6u, (texCoordIter++)->GetId());
}


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
TEST_F(TextureTest, SortedAdd)
{
  Point* pPoint = TextureTest::NewPoint(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0));
  
  TextureTest::SortedAddTextureCoordinate(pPoint, 4u, sc<Flt>(0.5), sc<Flt>(0.05));
  TextureTest::SortedAddTextureCoordinate(pPoint, 1u, sc<Flt>(0.2), sc<Flt>(0.02));
  TextureTest::SortedAddTextureCoordinate(pPoint, 5u, sc<Flt>(0.6), sc<Flt>(0.06));
  TextureTest::SortedAddTextureCoordinate(pPoint, 0u, sc<Flt>(0.1), sc<Flt>(0.01));
  TextureTest::SortedAddTextureCoordinate(pPoint, 6u, sc<Flt>(0.7), sc<Flt>(0.07));
  TextureTest::SortedAddTextureCoordinate(pPoint, 3u, sc<Flt>(0.4), sc<Flt>(0.04));
  TextureTest::SortedAddTextureCoordinate(pPoint, 2u, sc<Flt>(0.3), sc<Flt>(0.03));
  
  ::Container<TextureCoordinate>::ConstIterator texCoordIter(pPoint->GetTextureCoordinates().begin());
  
  EXPECT_EQ(0u, (texCoordIter++)->GetId());
  EXPECT_EQ(1u, (texCoordIter++)->GetId());
  EXPECT_EQ(2u, (texCoordIter++)->GetId());
  EXPECT_EQ(3u, (texCoordIter++)->GetId());
  EXPECT_EQ(4u, (texCoordIter++)->GetId());
  EXPECT_EQ(5u, (texCoordIter++)->GetId());
  EXPECT_EQ(6u, (texCoordIter++)->GetId());
}


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
TEST_F(TextureTest, SortedAddByRange)
{
  Point* pPoint = TextureTest::NewPoint(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0));
  
  TextureTest::SortedAddTextureCoordinateByRange(pPoint, 4u, sc<Flt>(0.5), sc<Flt>(0.05));
  TextureTest::SortedAddTextureCoordinateByRange(pPoint, 1u, sc<Flt>(0.2), sc<Flt>(0.02));
  TextureTest::SortedAddTextureCoordinateByRange(pPoint, 5u, sc<Flt>(0.6), sc<Flt>(0.06));
  TextureTest::SortedAddTextureCoordinateByRange(pPoint, 0u, sc<Flt>(0.1), sc<Flt>(0.01));
  TextureTest::SortedAddTextureCoordinateByRange(pPoint, 6u, sc<Flt>(0.7), sc<Flt>(0.07));
  TextureTest::SortedAddTextureCoordinateByRange(pPoint, 3u, sc<Flt>(0.4), sc<Flt>(0.04));
  TextureTest::SortedAddTextureCoordinateByRange(pPoint, 2u, sc<Flt>(0.3), sc<Flt>(0.03));
  
  ::Container<TextureCoordinate>::ConstIterator texCoordIter(pPoint->GetTextureCoordinates().begin());
  
  EXPECT_EQ(0u, (texCoordIter++)->GetId());
  EXPECT_EQ(1u, (texCoordIter++)->GetId());
  EXPECT_EQ(2u, (texCoordIter++)->GetId());
  EXPECT_EQ(3u, (texCoordIter++)->GetId());
  EXPECT_EQ(4u, (texCoordIter++)->GetId());
  EXPECT_EQ(5u, (texCoordIter++)->GetId());
  EXPECT_EQ(6u, (texCoordIter++)->GetId());
}


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
TEST_F(TextureTest, SortedAddRemove2)
{
  Point* pPoint = TextureTest::NewPoint(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0));
  
  TextureTest::SortedAddTextureCoordinate(pPoint, 4u, sc<Flt>(0.5), sc<Flt>(0.05));
  TextureTest::SortedAddTextureCoordinate(pPoint, 1u, sc<Flt>(0.2), sc<Flt>(0.02));
  TextureTest::SortedAddTextureCoordinate(pPoint, 5u, sc<Flt>(0.6), sc<Flt>(0.06));
  TextureTest::SortedAddTextureCoordinate(pPoint, 0u, sc<Flt>(0.1), sc<Flt>(0.01));
  TextureTest::SortedAddTextureCoordinate(pPoint, 2u, sc<Flt>(0.3), sc<Flt>(0.03));
  TextureTest::SortedAddTextureCoordinate(pPoint, 6u, sc<Flt>(0.7), sc<Flt>(0.07));
  TextureTest::SortedAddTextureCoordinate(pPoint, 3u, sc<Flt>(0.4), sc<Flt>(0.04));
  
  ::Container<TextureCoordinate>::ConstIterator texCoordIter(pPoint->GetTextureCoordinates().begin());
  
  
  ++texCoordIter;
  ++texCoordIter;
  TextureTest::RemoveAndDisposeTextureCoordinate(*texCoordIter, pPoint);
  
  texCoordIter = pPoint->GetTextureCoordinates().begin();
  ++texCoordIter;
  ++texCoordIter;
  ++texCoordIter;
  TextureTest::RemoveAndDisposeTextureCoordinate(*texCoordIter, pPoint);
  
  texCoordIter = pPoint->GetTextureCoordinates().begin();
  EXPECT_EQ(0u, (texCoordIter++)->GetId());
  EXPECT_EQ(1u, (texCoordIter++)->GetId());
  EXPECT_EQ(3u, (texCoordIter++)->GetId());
  EXPECT_EQ(5u, (texCoordIter++)->GetId());
  EXPECT_EQ(6u, (texCoordIter++)->GetId());
}


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
TEST_F(TextureTest, SortedAddRemove2ReAdd1)
{
  Point* pPoint = TextureTest::NewPoint(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0));
  
  TextureTest::SortedAddTextureCoordinate(pPoint, 4u, sc<Flt>(0.5), sc<Flt>(0.05));
  TextureTest::SortedAddTextureCoordinate(pPoint, 1u, sc<Flt>(0.2), sc<Flt>(0.02));
  TextureTest::SortedAddTextureCoordinate(pPoint, 5u, sc<Flt>(0.6), sc<Flt>(0.06));
  TextureTest::SortedAddTextureCoordinate(pPoint, 0u, sc<Flt>(0.1), sc<Flt>(0.01));
  TextureTest::SortedAddTextureCoordinate(pPoint, 2u, sc<Flt>(0.3), sc<Flt>(0.03));
  TextureTest::SortedAddTextureCoordinate(pPoint, 6u, sc<Flt>(0.7), sc<Flt>(0.07));
  TextureTest::SortedAddTextureCoordinate(pPoint, 3u, sc<Flt>(0.4), sc<Flt>(0.04));
  
  ::Container<TextureCoordinate>::ConstIterator texCoordIter(pPoint->GetTextureCoordinates().begin());
  
  
  ++texCoordIter;
  ++texCoordIter;
  TextureTest::RemoveAndDisposeTextureCoordinate(*texCoordIter, pPoint);
  
  texCoordIter = pPoint->GetTextureCoordinates().begin();
  ++texCoordIter;
  ++texCoordIter;
  ++texCoordIter;
  TextureTest::RemoveAndDisposeTextureCoordinate(*texCoordIter, pPoint);
  
  TextureTest::SortedAddTextureCoordinate(pPoint, 2u, sc<Flt>(0.3), sc<Flt>(0.03));
  
  texCoordIter = pPoint->GetTextureCoordinates().begin();
  EXPECT_EQ(0u, (texCoordIter++)->GetId());
  EXPECT_EQ(1u, (texCoordIter++)->GetId());
  EXPECT_EQ(2u, (texCoordIter++)->GetId());
  EXPECT_EQ(3u, (texCoordIter++)->GetId());
  EXPECT_EQ(5u, (texCoordIter++)->GetId());
  EXPECT_EQ(6u, (texCoordIter++)->GetId());
}
#endif
