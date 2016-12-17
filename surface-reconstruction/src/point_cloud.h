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

#ifndef SURFACE_RECONSTRUCTION__POINT_CLOUD_H_
#define SURFACE_RECONSTRUCTION__POINT_CLOUD_H_

#include <cstdlib>

#include <algorithm>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#endif

#include <common/aabb.h>
#include <common/logging.h>

#include "data_types.h"
#include "point.h"
#include "io/data_storage.h"


////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
class PointCloud
{
public:
  typedef boost::shared_ptr<PointCloud> SharedPtr;
  
  PointCloud()
  : fDirty(true)
  , fIsAvailable(false)
  {
    this->fRng.seed(0);
    this->fRngCallable = boost::bind(&PointCloud::Random, this, _1);
  }
  
  Point* NewPoint(const Position& position
#ifndef IGNORE_COLOUR
                  ,
                  Point::ColorComponent red =
                    static_cast<Point::ColorComponent>(1.0),
                  Point::ColorComponent green =
                    static_cast<Point::ColorComponent>(1.0),
                  Point::ColorComponent blue =
                    static_cast<Point::ColorComponent>(1.0)
#endif
                  , Flt u1 = sc<Flt>(-1.0)
                  , Flt v1 = sc<Flt>(-1.0)
                  , Flt u2 = sc<Flt>(-1.0)
                  , Flt v2 = sc<Flt>(-1.0)
                  , Flt u3 = sc<Flt>(-1.0)
                  , Flt v3 = sc<Flt>(-1.0)
                  , Flt u4 = sc<Flt>(-1.0)
                  , Flt v4 = sc<Flt>(-1.0)
                  , Flt u5 = sc<Flt>(-1.0)
                  , Flt v5 = sc<Flt>(-1.0)
                  , Flt u6 = sc<Flt>(-1.0)
                  , Flt v6 = sc<Flt>(-1.0)
                  , Flt u7 = sc<Flt>(-1.0)
                  , Flt v7 = sc<Flt>(-1.0))
  {
    bool insideObb = false;
    BOOST_FOREACH(const AABB_T<Flt>& obb, this->fOutlierBoundingBoxes)
    {
      if (obb.Contains(position))
      {
        insideObb = true;
        break;
      }
    }
    if (insideObb)
    {
      return NULL;
    }
    
#ifndef IGNORE_COLOUR
    this->fPoints.push_back(new Point(position, red, green, blue,
                                      u1, v1,
                                      u2, v2,
                                      u3, v3,
                                      u4, v4,
                                      u5, v5,
                                      u6, v6,
                                      u7, v7));
#else
    this->fPoints.push_back(new Point(position,
                                      u1, v1,
                                      u2, v2,
                                      u3, v3,
                                      u4, v4,
                                      u5, v5,
                                      u6, v6,
                                      u7, v7));
#endif
    this->fBoundingBox.ResizeToIncludePoint(position);
    this->fDirty = true;
    return this->fPoints.back();
  }
  
  Point* GetPoint(Point::Container::size_type index)
  {
    return this->fPoints[index];
  }
  
  const AABB_T<Flt>& GetBoundingBox() const { return this->fBoundingBox; }
  
  void LoadOBB(const std::string& obbFileName)
  {
    std::ifstream obbFile(obbFileName.c_str());
    if (obbFile.is_open())
    {
      this->fOutlierBoundingBoxes.clear();
      
      std::string inputLine;
      std::getline(obbFile, inputLine); // num of OBBs
      unsigned int numOfObbs = std::atoi(inputLine.c_str());
      
      boost::char_separator<char> sep(" ");
      typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;
      for (unsigned int i = 0; i < numOfObbs; ++i)
      {
        std::getline(obbFile, inputLine);
        Tokenizer tokens(inputLine, sep);
        Tokenizer::iterator tokenIter = tokens.begin();
        const Flt xmin = sc<Flt>(std::atof((tokenIter++)->c_str()));
        const Flt ymin = sc<Flt>(std::atof((tokenIter++)->c_str()));
        const Flt zmin = sc<Flt>(std::atof((tokenIter++)->c_str()));
        const Flt xmax = sc<Flt>(std::atof((tokenIter++)->c_str()));
        const Flt ymax = sc<Flt>(std::atof((tokenIter++)->c_str()));
        const Flt zmax = sc<Flt>(std::atof((tokenIter++)->c_str()));
        this->fOutlierBoundingBoxes.push_back(AABB_T<Flt>(xmin, ymin, zmin,
                                                          xmax, ymax, zmax));
      }
      obbFile.close();
      
      LOG_INFO("Loaded " << this->fOutlierBoundingBoxes.size() << " OBBs.");
    }
  }
  
  void CleanWithOBB()
  {
  }
               
  /// \brief Loads a point cloud from a data source
  void Load(const io::DataStorage<PointCloud>& dataStorage)
  {
    dataStorage.Load(this);

    
    this->InitSampleIter();
    this->fDirty = true;
  }
  
  void Clear()
  {
    this->fIsAvailable = false;
    BOOST_FOREACH(Point* pPoint, this->fPoints)
    {
      delete pPoint;
    }
    this->fBoundingBox = AABB_T<Flt>();
    this->fPoints.clear();
    this->fDirty = true;
  }
  
  /// \brief Returns the number of points in the point cloud
  Point::Container::size_type GetNumOfPoints() const 
  { 
    return this->fPoints.size(); 
  }
  
  Point::Range GetPoints() { return this->fPoints; }
  Point::ConstRange GetPoints() const { return this->fPoints; }
  
  Point* GetNext()
  {
    ++this->fCurrentSampleIter;
    if (this->fCurrentSampleIter == this->fPoints.end())
    {
      this->Shuffle();
    }
    return *(this->fCurrentSampleIter);
  }
  
  bool IsDirty() const { return this->fDirty; }
  void RemoveDirty() { this->fDirty = false; }
  void MakeDirty() { this->fDirty = true; }
  
  void Shuffle()
  {
    boost::range::random_shuffle(this->fPoints, this->fRngCallable);
    this->InitSampleIter();
  }
  
  void InitSampleIter()
  {
    this->fCurrentSampleIter = this->fPoints.begin();
  }
  
  
  const std::vector<AABB_T<Flt> >& GetOutlierBoundingBoxes() const
  {
    return this->fOutlierBoundingBoxes;
  }
  
#ifndef IGNORE_COLOUR
  void SetPointColour(Flt r, Flt g, Flt b, Point* pPoint)
  {
    pPoint->SetColour(r, g, b);
  }
#endif
  
  bool IsAvailable() const { return this->fIsAvailable; }
  void SetIsAvailable() { this->fIsAvailable = true; }
  
private:
  
  ptrdiff_t Random (ptrdiff_t i)
  {
    return this->fRng() % i;
  }
  
  Point::Container fPoints;
  AABB_T<Flt> fBoundingBox;
  
  Point::Container::iterator fCurrentSampleIter;
  boost::random::mt11213b fRng;
  boost::function<ptrdiff_t(ptrdiff_t)> fRngCallable;
  
  bool fDirty;
  bool fIsAvailable;
  
  std::vector<AABB_T<Flt> > fOutlierBoundingBoxes;
};  // class PointCloud

#endif  // #ifndef SURFACE_RECONSTRUCTION__POINT_CLOUD_H_
