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

#include <cassert>

#include <fstream>
#include <string>

#ifndef Q_MOC_RUN
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/date_time.hpp>
#endif

#include <common/logging.h>
#include <common/unused.h>


#include "io/triangle_mesh_file.h"

#include "data_types.h"
#include "input_adapter.h"
#include "program_options.h"
#include "surface_reconstruction.h"



////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
SurfaceReconstruction::SurfaceReconstruction
()
: fpPointCloud()
, fpDensePointCloud()
, fpTriangleMesh()
, fIterationsDone(0ul)
, fpLastSignal(NULL)
, fpLastClosestVertices0(NULL)
, fpLastClosestVertices1(NULL)
, fInitializedSGNG(false)
, fTargetNumVertices(0u)
{
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::Init
()
{
  namespace bfs = boost::filesystem;
  
  this->fpPointCloud.reset(new PointCloud);
  this->fpDensePointCloud.reset(new PointCloud);
  this->fpTriangleMesh.reset(new TriangleMesh);
  this->fpTextures.reset(new Textures);
  
  const std::string obbFile(ProgramOptions::GetOutlierBoundingFile());
  
  if (obbFile.size() != 0)
  {
    this->fpPointCloud->LoadOBB(obbFile);
    this->fpDensePointCloud->LoadOBB(obbFile);
  }
  
  this->fRemotePrefix = bfs::path(ProgramOptions::GetRemotePrefix());
  this->fLocalPrefix = bfs::path(ProgramOptions::GetLocalPrefix());
  
  this->LoadInputFile(ProgramOptions::GetInputFile());
  
  // triangle mesh storage -- file
  const std::string triangleMeshFile(ProgramOptions::GetMeshFile());
}




////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::InitSGNG()
{
  if (this->fIterationsDone > 0)
  {
    LOG_ERROR("Cannot initialize an sGNG network that has already learnt!");
    LOG_ERROR("Ignoring initSGNG command.");
    return;
  }
  
  if (this->fpPointCloud->GetNumOfPoints() < 2 &&
      this->fpDensePointCloud->GetNumOfPoints() < 2)
  {
    LOG_ERROR("Cannot initialize an sGNG network from less than 2 points!");
    LOG_ERROR("Ignoring initSGNG command.");
    return;
  }
  
  // delete all triangles, edges, vertices
  this->fpTriangleMesh->DeleteAllTriangles();
  this->fpTriangleMesh->DeleteAllEdges();
  this->fpTriangleMesh->DeleteAllVertices();
  
  boost::random::mt11213b rng;
  const unsigned int numSparsePoints =
    this->GetPointCloud()->GetNumOfPoints();
  const unsigned int numDensePoints =
    this->GetDensePointCloud()->GetNumOfPoints();
  const unsigned int randomNumber = rng() % (numSparsePoints + numDensePoints);
  PointCloud::SharedPtr pPointCloud(this->GetPointCloud());
  
  if (randomNumber >= numSparsePoints)
  {
    pPointCloud = this->GetDensePointCloud();
  }
  
  // initialize sGNG
  // get two initial points from point cloud
  const Point* const pPoint0(pPointCloud->GetNext());
  const Point* const pPoint1(pPointCloud->GetNext());
  
  // add two vertices
  Vertex* pV0 = this->fpTriangleMesh->NewVertex(pPoint0->GetPosition());
  pV0->SetNormal(pPoint0->GetNormal());
  assert(pV0 != NULL);
#ifndef IGNORE_COLOUR
  pV0->SetRed(pPoint0->GetRed());
  pV0->SetGreen(pPoint0->GetGreen());
  pV0->SetBlue(pPoint0->GetBlue());
#endif
  
  Vertex* pV1 = this->fpTriangleMesh->NewVertex(pPoint1->GetPosition());
  pV1->SetNormal(pPoint1->GetNormal());
  assert(pV1 != NULL);
#ifndef IGNORE_COLOUR
  pV1->SetRed(pPoint1->GetRed());
  pV1->SetGreen(pPoint1->GetGreen());
  pV1->SetBlue(pPoint1->GetBlue());
#endif

  this->fpTriangleMesh->SetAverageEdgeLength((pV1->GetPosition() - pV0->GetPosition()).GetSquaredLength());
  
#ifndef NO_TEXTURES
  typedef ::Container<TextureCoordinate> CTC;
  CTC::ConstIterator texCoordsP0Iter(pPoint0->GetTextureCoordinates().begin());
  CTC::ConstIterator texCoordsP1Iter(pPoint1->GetTextureCoordinates().begin());
  
  while (texCoordsP0Iter != pPoint0->GetTextureCoordinates().end())
  {
    TextureCoordinate::SortedAdd(*(new TextureCoordinate(*texCoordsP0Iter)),
                                 pV0->GetTextureCoordinates());
    ++texCoordsP0Iter;
  }
  while (texCoordsP1Iter != pPoint1->GetTextureCoordinates().end())
  {
    TextureCoordinate::SortedAdd(*(new TextureCoordinate(*texCoordsP1Iter)),
                                 pV1->GetTextureCoordinates());
    ++texCoordsP1Iter;
  }
#endif
  
  // add edge
  Edge* pEdge = this->fpTriangleMesh->NewEdge(pV0, pV1);
  UNUSED(pEdge);
  assert(pEdge != NULL);
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::LoadInputFile
(const std::string& inputFile)
{
  namespace bfs = boost::filesystem;

  const bfs::path inputFilePath(inputFile);
  
  if (!inputFilePath.empty() &&
      inputFilePath.is_relative() &&
      !this->fRemotePrefix.empty())
  {
    this->fPointPath = inputFilePath.parent_path();
    
    if (!bfs::exists(this->fLocalPrefix / this->fPointPath))
    {
      bfs::create_directories(this->fLocalPrefix / this->fPointPath);
    }
    
    // copy image data if necessary
    if (inputFilePath.extension().compare(".nvm") == 0)
    {
      if (bfs::exists(this->fRemotePrefix / inputFilePath))
      {
        bfs::copy_file(this->fRemotePrefix / inputFilePath,
                       this->fLocalPrefix / inputFilePath,
                       bfs::copy_option::overwrite_if_exists);
        
        std::ifstream nvmStream(
          (this->fLocalPrefix / inputFilePath).c_str());
        if (nvmStream.is_open())
        {
          std::string inputLine;
          std::getline(nvmStream, inputLine);
          std::getline(nvmStream, inputLine);
          std::getline(nvmStream, inputLine);
          unsigned int numOfCams = std::atoi(inputLine.c_str());
          for (unsigned int c = 0; c < numOfCams; ++c)
          {
            std::getline(nvmStream, inputLine);
            const unsigned int firstSpace = inputLine.find_first_of(" \t");
            const bfs::path camFilePath(inputLine.substr(0, firstSpace));
            if (!bfs::exists(
              this->fLocalPrefix / this->fPointPath / camFilePath))
            {
              if (!bfs::exists(this->fLocalPrefix /
                               this->fPointPath /
                               camFilePath.parent_path()))
              {
                bfs::create_directories(this->fLocalPrefix /
                                        this->fPointPath /
                                        camFilePath.parent_path());
              }
              if (bfs::exists(this->fRemotePrefix /
                              this->fPointPath /
                              camFilePath.parent_path()))
              {
                try {
                bfs::copy_file(this->fRemotePrefix /
                               this->fPointPath /
                               camFilePath,
                               this->fLocalPrefix /
                               this->fPointPath /
                               camFilePath);
                }
                catch (const std::exception& e)
                {
                  std::cout << e.what() << std::endl;
                }
              }
            }
          }
          nvmStream.close();
        }
        this->fInputData.Reset((this->fLocalPrefix / inputFilePath).string());
        this->ReloadInputFile();
      }
    } // if (inputFilePath.extension().compare(".nvm") == 0)
    else if (inputFilePath.extension().compare(".dense") == 0)
    {
      if (bfs::exists(this->fRemotePrefix / inputFilePath))
      {
        if (!bfs::exists(this->fLocalPrefix / inputFilePath))
        {
          bfs::create_directories(this->fLocalPrefix / inputFilePath);
        }
        
        bfs::directory_iterator endIter;
        for(bfs::directory_iterator dirIter(
              this->fRemotePrefix / inputFilePath);
            dirIter != endIter;
            ++dirIter)
        {
          try {
          bfs::copy_file(dirIter->path(),
                         this->fLocalPrefix /
                           this->fPointPath /
                             inputFilePath.filename() /
                               dirIter->path().filename(),
                         bfs::copy_option::overwrite_if_exists);
          }
          catch (const std::exception& e)
          {
            std::cout << e.what() << std::endl;
          }
        }
        
        this->fDenseInputData.Reset(
          (this->fLocalPrefix / inputFilePath).string());
        this->ReloadDenseInputFile();
      }
    } // else if (inputFilePath.extension().compare(".dense") == 0)
  }
  else
  {
    this->fInputData.Reset(inputFile);
    this->ReloadInputFile();
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::ReloadInputFile
()
{
  namespace bfs = boost::filesystem;
  if (this->fInputData.IsValid())
  {
    this->LockPoints();
    this->LockTextures();
    
    // remove old data
    this->fpPointCloud->Clear();
    this->fpTextures->Clear();

    // load new data
    InputAdapter adapter(this->fpPointCloud.get(), this->fpTextures.get());
    this->fInputData.Load(&adapter);
    
    LOG_INFO("Num of points loaded: " << this->fpPointCloud->GetNumOfPoints());
    
    // clean
    this->fpPointCloud->CleanWithOBB();
    LOG_INFO("Num of after clean  : " << this->fpPointCloud->GetNumOfPoints());
    
    this->fpPointCloud->Shuffle();
    this->fpPointCloud->MakeDirty();
    
    if ((this->fIterationsDone == 0) &&
        (this->fpPointCloud->GetNumOfPoints() > 1))
    {
      this->InitSGNG();
    }
    this->fpPointCloud->SetIsAvailable();
    
    this->UnlockTextures();
    this->UnlockPoints();
    
    this->LockMesh();
    
    this->UnlockMesh();
    
    this->fpLastClosestVertices0 = NULL;
    this->fpLastClosestVertices1 = NULL;
    this->fpLastSignal = NULL;
    
    const Flt pvr = ProgramOptions::PointVertexRatio();
    const unsigned int np = this->fpPointCloud->GetNumOfPoints();
    const unsigned int nv = ProgramOptions::NumVertices();
    this->fTargetNumVertices =
      nv > 0 ? nv : static_cast<unsigned int>(static_cast<Flt>(np) / pvr);
    std::cout << "Target num vertices: " << this->fTargetNumVertices << std::endl;
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::ReloadDenseInputFile
()
{
  if (this->fDenseInputData.IsValid())
  {
    this->LockPoints();
    
    // remove old data
    this->fpDensePointCloud->Clear();
    
    // load new data
    InputAdapter adapter(this->fpDensePointCloud.get(), NULL);
    this->fDenseInputData.Load(&adapter);
    
    LOG_INFO("Num of points loaded: " << this->fpDensePointCloud->GetNumOfPoints());
    
    // clean
    this->fpDensePointCloud->CleanWithOBB();
    LOG_INFO("Num of after clean  : " << this->fpDensePointCloud->GetNumOfPoints());
    
    this->fpDensePointCloud->Shuffle();
    this->fpDensePointCloud->MakeDirty();
    
    if ((this->fIterationsDone == 0) &&
        (this->fpDensePointCloud->GetNumOfPoints() > 1))
    {
      this->InitSGNG();
    }
    this->fpDensePointCloud->SetIsAvailable();
    
    this->UnlockPoints();
    
    this->LockMesh();
    this->UnlockMesh();
  }
  
  this->fpLastClosestVertices0 = NULL;
  this->fpLastClosestVertices1 = NULL;
  this->fpLastSignal = NULL;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::Statistics
()
{
  this->LockMesh();

#ifndef NO_TEXTURES
  // textures per point
  unsigned int minNumOfTexturesPerPoint = 0u - 1u;
  unsigned int maxNumOfTexturesPerPoint = 0u;
  unsigned int totalNumberOfTextures = 0u;

  BOOST_FOREACH(const Point* pPoint, this->fpPointCloud->GetPoints())
  {
    unsigned int count = 0u;
    BOOST_FOREACH(const TextureCoordinate& texCoord,
                  pPoint->GetTextureCoordinates())
    {
      UNUSED(texCoord);
      ++count;
    }
    minNumOfTexturesPerPoint = std::min(count, minNumOfTexturesPerPoint);
    maxNumOfTexturesPerPoint = std::max(count, maxNumOfTexturesPerPoint);
    totalNumberOfTextures += count;
  }
  
  const float avgTexturesPerPoint =
    static_cast<float>(totalNumberOfTextures) /
      static_cast<float>(this->fpPointCloud->GetNumOfPoints());
  
  unsigned int standardDeviationAccumulator = 0u;
  BOOST_FOREACH(const Point* pPoint, this->fpPointCloud->GetPoints())
  {
    unsigned int count = 0u;
    BOOST_FOREACH(const TextureCoordinate& texCoord,
                  pPoint->GetTextureCoordinates())
    {
      UNUSED(texCoord);
      ++count;
    }
    standardDeviationAccumulator +=
    (avgTexturesPerPoint - count) * (avgTexturesPerPoint - count);
  }
  
  const float standardDeviation =
  sqrtf(static_cast<float>(standardDeviationAccumulator) /
        static_cast<float>(this->fpPointCloud->GetNumOfPoints() - 1));
  
  LOG_INFO("Min. tex. cand. per pt.: " << minNumOfTexturesPerPoint);
  LOG_INFO("Max. tex. cand. per pt.: " << maxNumOfTexturesPerPoint);
  LOG_INFO("Avg. tex. cand. per pt.: " << avgTexturesPerPoint);
  LOG_INFO("Stddev.                : " << standardDeviation);
#endif
  
  this->UnlockMesh();
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void 
SurfaceReconstruction::SaveTriangleMesh
()
{
  this->fpMeshStorage->Delete(this->fpTriangleMesh.get());
  this->fpMeshStorage->Save(this->fpTriangleMesh.get());
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::InitForRenderedReconstruction
()
{
  this->fpReconstructionWorker.reset(new ReconstructionWorker(this));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::DoRenderedReconstruction
()
{
  this->fpReconstructionWorker->Run(NULL);
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::ReconstructStandalone
()
{
  this->fpReconstructionWorker.reset(new ReconstructionWorker(this));
  this->fpReconstructionWorker->Run();
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::Stop
()
{
  if (this->fpReconstructionWorker.get() != NULL)
  {
    this->fpReconstructionWorker->Stop();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
std::string SurfaceReconstruction::CreateTimestampStr()
{
  std::ostringstream timestamp;
  boost::local_time::local_date_time t = 
    boost::local_time::local_sec_clock::local_time(
    boost::local_time::time_zone_ptr());
  boost::local_time::local_time_facet* lf(
    new boost::local_time::local_time_facet("%Y-%m-%dT%H%M%SZ"));
  timestamp.imbue(std::locale(timestamp.getloc(),lf));
  timestamp << t;
  return timestamp.str();
}



////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::Reset
()
{
  this->LockMesh();
  this->fpTriangleMesh->ResetVerticesEdgesTriangles();
  this->InitSGNG();
  this->UnlockMesh();
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
SurfaceReconstruction::ExportObj
(const std::string& filename)
{
  const boost::filesystem::path& filePath(filename);
  
  //  std::cout << filePath << std::endl;
  namespace bfs = boost::filesystem;
  
  if (!bfs::exists(filePath.parent_path()))
  {
    bfs::create_directories(filePath.parent_path());
  }
  
  // store obj
  const bfs::path objFilePath(bfs::path(filePath).replace_extension(".obj"));
  std::ofstream obj(objFilePath.c_str());
  
  if (obj.is_open())
  {
    std::stringstream vtxStream;
    std::stringstream normalStream;
    std::stringstream texStream;
    std::stringstream triStream;
    
    this->LockMesh();
    this->LockTextures();
    
    const TriangleMesh* const pMesh = this->GetTriangleMesh().get();
    
    std::list<const Vertex*> verticesInUse;
    BOOST_FOREACH(const Triangle& tri, pMesh->GetTriangles())
    {
      verticesInUse.push_back(tri.GetV0());
      verticesInUse.push_back(tri.GetV1());
      verticesInUse.push_back(tri.GetV2());
    }
    verticesInUse.sort();
    verticesInUse.unique();
    
    // vertices and normals
    unsigned int currVtxId = 1u;
    std::map<const Vertex* const, unsigned int> vtxIdMap;
    BOOST_FOREACH(const Vertex* const pVtx, verticesInUse)
    {
      vtxStream << "v ";
      vtxStream << pVtx->GetPosition().GetX() << " ";
      vtxStream << pVtx->GetPosition().GetY() << " ";
      vtxStream << pVtx->GetPosition().GetZ() << std::endl;
      
      vtxIdMap[pVtx] = currVtxId++;
    }
    
    // triangles and tex coords
    BOOST_FOREACH(const Triangle& tri, pMesh->GetTriangles())
    {
      bool reorient = tri.NeedsReorient();
      
      triStream << "f ";
      
      if (reorient)
      {
        triStream << vtxIdMap[tri.GetV2()];
      }
      else
      {
        triStream << vtxIdMap[tri.GetV0()];
      }
      triStream << "/";
      triStream << "/";
      //triStream << vtxIdMap[tri.GetV0()];

      triStream << " ";
      
      triStream << vtxIdMap[tri.GetV1()];
      triStream << "/";
      triStream << "/";
      //triStream << vtxIdMap[tri.GetV1()];
      
      triStream << " ";
      
      if (reorient)
      {
        triStream << vtxIdMap[tri.GetV0()];
      }
      else
      {
        triStream << vtxIdMap[tri.GetV2()];
      }
      triStream << "/";
      triStream << "/";
      //triStream << vtxIdMap[tri.GetV2()];
      
      triStream << std::endl;
    }
    
    this->UnlockTextures();
    this->UnlockMesh();
    
    obj << "# Export from sGNG" << std::endl;
    obj << "# Input: " << this->fInputData.GetFileName() << std::endl;
    obj << "# Num points: " << this->fpPointCloud->GetNumOfPoints() << std::endl;
    obj << "# beta: " << this->fpReconstructionWorker->GetBeta() << std::endl;
    obj << "# eta: " << this->fpReconstructionWorker->GetEta() << std::endl;
    obj << "# lambda: " << this->fpReconstructionWorker->GetLambda() << std::endl;
    obj << "# vtx inact. thd.: " << this->fpReconstructionWorker->GetVertexInactivityThreshold() << std::endl;
    obj << "# max edge penalty: " << this->fpReconstructionWorker->GetMaxEdgePenalty() << std::endl;
    obj << "# max len^2: " << this->fpReconstructionWorker->GetMaxLen2() << std::endl;
    obj << "# time: " << this->fpReconstructionWorker->GetTime() << std::endl;
    obj << std::endl;
    obj << vtxStream.str();
    obj << texStream.str();
    obj << normalStream.str();
    obj << triStream.str();
    obj << std::endl;
    
    obj.close();
  }
#ifndef NO_TEXTURES
  this->ExportTexturedObj(filename);
#endif
#ifndef IGNORE_COLOR
  this->ExportColoredPly(filename);
#endif
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
#ifndef NO_TEXTURES
void
SurfaceReconstruction::ExportTexturedObj
(const std::string& filename)
{
  boost::filesystem::path filePath(filename);
  filePath.replace_extension("");
  namespace bfs = boost::filesystem;
  
  if (!bfs::exists(filePath))
  {
    bfs::create_directories(filePath);
  }
  
  struct VertexNormalTexCoord
  {
    float v0x, v0y, v0z;
    float v1x, v1y, v1z;
    float v2x, v2y, v2z;
    float n0x, n0y, n0z;
    float n1x, n1y, n1z;
    float n2x, n2y, n2z;
    float u0, v0;
    float u1, v1;
    float u2, v2;
  };
  
  // Compute triangle normals
  BOOST_FOREACH(Triangle& tri,
                this->GetTriangleMesh()->GetTriangles())
  {
    const Vector p0 = tri.GetV0()->GetPosition();
    const Vector p1 = tri.GetV1()->GetPosition();
    const Vector p2 = tri.GetV2()->GetPosition();
    const Vector normal(crossProduct(p1 - p0, p2 - p0));
    tri.SetNormal(normalize(normal));
  }
  
  // Compute vertex normals
  BOOST_FOREACH(Vertex& vtx,
                this->GetTriangleMesh()->GetVertices())
  {
    Vector vtxNormal(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0));
    Vector firstNormal(sc<Flt>(0.0), sc<Flt>(0.0), sc<Flt>(0.0));
    bool firstNormalInvalid = true;
    unsigned int normalCount = 0u;
    BOOST_FOREACH(const Edge* pEdge, Adjacent<Edge>::Around(&vtx))
    {
      BOOST_FOREACH(const Triangle* pTri, Adjacent<Triangle>::Around(pEdge))
      {
        if (firstNormalInvalid)
        {
          firstNormalInvalid = false;
          firstNormal = pTri->GetNormal();
        }
        if (dotProduct(pTri->GetNormal(), firstNormal) < sc<Flt>(0.0))
        {
          vtxNormal += pTri->GetNormal();
        }
        else
        {
          vtxNormal -= pTri->GetNormal();
        }
        ++normalCount;
      }
    }
    vtxNormal *= sc<Flt>(1.0) / sc<Flt>(normalCount);
    vtx.SetNormal(vtxNormal);
  }
  
  
  std::vector<std::vector<VertexNormalTexCoord> > texturedTriangles;
  texturedTriangles.resize(this->GetTextures()->GetNumOfTextures());
  
  std::map<unsigned int, const Texture*> textureID2Texture;
  BOOST_FOREACH(const Texture& texture,
                this->GetTextures()->GetTextures())
  {
    const unsigned int texId = texture.GetId();
    textureID2Texture[texId] = &texture;
  }
  
  unsigned int texwidth = ProgramOptions::GetTextureWidth();
  unsigned int texheight = ProgramOptions::GetTextureHeight();
  unsigned int i = 0u;
  std::map<unsigned int, unsigned int> texturesUsed;
  BOOST_FOREACH(const Triangle& tri,
                this->GetTriangleMesh()->GetTriangles())
  {
    const Vector barycenter((sc<Flt>(1.0) / sc<Flt>(3.0)) *
                            (tri.GetV0()->GetPosition() +
                             tri.GetV1()->GetPosition() +
                             tri.GetV2()->GetPosition()));
    // determine texture
    typedef ::Container<TextureCoordinate> CTC;
    CTC::ConstIterator texCoordsV0Iter(
      tri.GetV0()->GetTextureCoordinates().begin());
    const CTC::ConstIterator texCoordsV0End(
      tri.GetV0()->GetTextureCoordinates().end());
    CTC::ConstIterator texCoordsV1Iter(
      tri.GetV1()->GetTextureCoordinates().begin());
    const CTC::ConstIterator texCoordsV1End(
      tri.GetV1()->GetTextureCoordinates().end());
    CTC::ConstIterator texCoordsV2Iter(
      tri.GetV2()->GetTextureCoordinates().begin());
    const CTC::ConstIterator texCoordsV2End(
      tri.GetV2()->GetTextureCoordinates().end());
    
    // these iterators store the selected texture coordinates
    CTC::ConstIterator texCoords0Iter(texCoordsV0Iter);
    CTC::ConstIterator texCoords1Iter(texCoordsV1Iter);
    CTC::ConstIterator texCoords2Iter(texCoordsV2Iter);
    
    float hOffset = 0.0f;
    float vOffset = 0.0f;
    unsigned int width = 1u;
    unsigned int height = 1u;
    
    unsigned int texNum = 0;
    {
      Flt maxDotP = -1.0f;
      while (texCoordsV0Iter != texCoordsV0End)
      {
        while ((texCoordsV1Iter != texCoordsV1End) &&
               texCoordsV1Iter->GetId() < texCoordsV0Iter->GetId())
        {
          ++texCoordsV1Iter;
        }
        while ((texCoordsV2Iter != texCoordsV2End) &&
               texCoordsV2Iter->GetId() < texCoordsV0Iter->GetId())
        {
          ++texCoordsV2Iter;
        }
        
        if ((texCoordsV1Iter->GetId() == texCoordsV0Iter->GetId()) &&
            (texCoordsV2Iter->GetId() == texCoordsV0Iter->GetId()))
        {
          const Flt dotPA = fabs(dotProduct(
                                            normalize(texCoordsV0Iter->GetCamPosition()-barycenter),
                                            tri.GetNormal()));
          const Flt dotPB = (dotProduct(
                                        normalize(barycenter - texCoordsV0Iter->GetCamPosition()),
                                        texCoordsV0Iter->GetCamDirection()));
          const Flt expect0 =
          sc<Flt>(texCoordsV0Iter->GetConfidence()) / sc<Flt>(tri.GetV0()->AdaptCount());
          const Flt expect1 =
          sc<Flt>(texCoordsV1Iter->GetConfidence()) / sc<Flt>(tri.GetV1()->AdaptCount());
          const Flt expect2 =
          sc<Flt>(texCoordsV2Iter->GetConfidence()) / sc<Flt>(tri.GetV2()->AdaptCount());
          const Flt expect = (expect0 * expect1 * expect2);
          
          if ((expect/expect == expect/expect) && (expect * dotPA * dotPB > maxDotP))
          {
            maxDotP = expect * dotPA * dotPB;
            texCoords0Iter = texCoordsV0Iter;
            texCoords1Iter = texCoordsV1Iter;
            texCoords2Iter = texCoordsV2Iter;
            width = texwidth;
            height = texheight;
            texNum = texCoordsV0Iter->GetId();
          }
        }
        
        ++texCoordsV0Iter;
      }
    }
    
    const Texture* const pTexture =
    textureID2Texture[texCoords0Iter->GetId()];
    
    // flip normals if necessary
    if (this->GetTextures()->GetNumOfTextures() > 0)
    {
      if (pTexture != NULL)
      {
        if (dotProduct(normalize(pTexture->GetCamPosition()-barycenter),
                       tri.GetV0()->GetNormal()) < 0.0f)
        {
          const_cast<Vertex*>(tri.GetV0())->SetNormal(-1.0f * tri.GetV0()->GetNormal());
        }
        if (dotProduct(normalize(pTexture->GetCamPosition()-barycenter),
                       tri.GetV1()->GetNormal()) < 0.0f)
        {
          const_cast<Vertex*>(tri.GetV1())->SetNormal(-1.0f * tri.GetV1()->GetNormal());
        }
        if (dotProduct(normalize(pTexture->GetCamPosition()-barycenter),
                       tri.GetV2()->GetNormal()) < 0.0f)
        {
          const_cast<Vertex*>(tri.GetV2())->SetNormal(-1.0f * tri.GetV2()->GetNormal());
        }
      }
      else
      {
        if (dotProduct(normalize(texCoordsV0Iter->GetCamPosition()-barycenter),
                       tri.GetV0()->GetNormal()) < 0.0f)
        {
          const_cast<Vertex*>(tri.GetV0())->SetNormal(-1.0f * tri.GetV0()->GetNormal());
        }
        if (dotProduct(normalize(texCoordsV1Iter->GetCamPosition()-barycenter),
                       tri.GetV1()->GetNormal()) < 0.0f)
        {
          const_cast<Vertex*>(tri.GetV1())->SetNormal(-1.0f * tri.GetV1()->GetNormal());
        }
        if (dotProduct(normalize(texCoordsV2Iter->GetCamPosition()-barycenter),
                       tri.GetV2()->GetNormal()) < 0.0f)
        {
          const_cast<Vertex*>(tri.GetV2())->SetNormal(-1.0f * tri.GetV2()->GetNormal());
        }
      }
    }
    
    const float u0 =
    ((pTexture != NULL) ? pTexture->GetU(tri.GetV0()->GetPosition()) : 0.0f);
    const float v0 =
    ((pTexture != NULL) ? pTexture->GetV(tri.GetV0()->GetPosition()) : 0.0f);
    const float u1 =
    ((pTexture != NULL) ? pTexture->GetU(tri.GetV1()->GetPosition()) : 0.0f);
    const float v1 =
    ((pTexture != NULL) ? pTexture->GetV(tri.GetV1()->GetPosition()) : 0.0f);
    const float u2 =
    ((pTexture != NULL) ? pTexture->GetU(tri.GetV2()->GetPosition()) : 0.0f);
    const float v2 =
    ((pTexture != NULL) ? pTexture->GetV(tri.GetV2()->GetPosition()) : 0.0f);
    
    VertexNormalTexCoord vertexNormalTexCoord;
    vertexNormalTexCoord.v0x = tri.GetV0()->GetPosition().GetX();
    vertexNormalTexCoord.v0y = tri.GetV0()->GetPosition().GetY();
    vertexNormalTexCoord.v0z = tri.GetV0()->GetPosition().GetZ();
    vertexNormalTexCoord.v1x = tri.GetV1()->GetPosition().GetX();
    vertexNormalTexCoord.v1y = tri.GetV1()->GetPosition().GetY();
    vertexNormalTexCoord.v1z = tri.GetV1()->GetPosition().GetZ();
    vertexNormalTexCoord.v2x = tri.GetV2()->GetPosition().GetX();
    vertexNormalTexCoord.v2y = tri.GetV2()->GetPosition().GetY();
    vertexNormalTexCoord.v2z = tri.GetV2()->GetPosition().GetZ();
    vertexNormalTexCoord.n0x = tri.GetV0()->GetNormal().GetX();
    vertexNormalTexCoord.n0x = tri.GetV0()->GetNormal().GetY();
    vertexNormalTexCoord.n0x = tri.GetV0()->GetNormal().GetZ();
    vertexNormalTexCoord.n1x = tri.GetV0()->GetNormal().GetX();
    vertexNormalTexCoord.n1x = tri.GetV0()->GetNormal().GetY();
    vertexNormalTexCoord.n1x = tri.GetV0()->GetNormal().GetZ();
    vertexNormalTexCoord.n2x = tri.GetV0()->GetNormal().GetX();
    vertexNormalTexCoord.n2x = tri.GetV0()->GetNormal().GetY();
    vertexNormalTexCoord.u0 = u0 / sc<Flt>(width);
    vertexNormalTexCoord.v0 = 1.0 - v0 / sc<Flt>(height);
    vertexNormalTexCoord.u1 = u1 / sc<Flt>(width);
    vertexNormalTexCoord.v1 = 1.0 - v1 / sc<Flt>(height);
    vertexNormalTexCoord.u2 = u2 / sc<Flt>(width);
    vertexNormalTexCoord.v2 = 1.0 - v2 / sc<Flt>(height);
    
    texturedTriangles[pTexture->GetId()].push_back(vertexNormalTexCoord);
  }
  
  
  
  
  const TriangleMesh* const pMesh = this->GetTriangleMesh().get();
  
  
  unsigned int id = 0;
  BOOST_FOREACH(const std::vector<VertexNormalTexCoord>& vnts, texturedTriangles)
  {
    if (vnts.size() > 0)
    {
      std::stringstream num;
      num << std::setfill('0') << std::setw(8) << id;
      // store obj
      const bfs::path objFilePath(bfs::path(filePath).string() +
                                  "/" + num.str() + ".obj");
      //      std::cout << objFilePath << std::endl;
      std::ofstream obj(objFilePath.c_str());
      
      int j = 1;
      if (obj.is_open())
      {
        
        //obj << "g  " << id << std::endl;
        BOOST_FOREACH(const VertexNormalTexCoord& vnt, vnts)
        {
          obj << "v  " << vnt.v0x << " " << vnt.v0y << " " << vnt.v0z << std::endl;
          //obj << "vn " << vnt.n0x << " " << vnt.n0y << " " << vnt.n0z << std::endl;
          obj << "vt " << vnt.u0 << " " << vnt.v0 << std::endl;
          obj << "v  " << vnt.v1x << " " << vnt.v1y << " " << vnt.v1z << std::endl;
          //obj << "vn " << vnt.n1x << " " << vnt.n1y << " " << vnt.n1z << std::endl;
          obj << "vt " << vnt.u1 << " " << vnt.v1 << std::endl;
          obj << "v  " << vnt.v2x << " " << vnt.v2y << " " << vnt.v2z << std::endl;
          //obj << "vn " << vnt.n2x << " " << vnt.n2y << " " << vnt.n2z << std::endl;
          obj << "vt " << vnt.u2 << " " << vnt.v2 << std::endl;
          obj << "f  " << j << "/" << j;// << "/" << j;
          obj << " " << j+1 << "/" << j+1;// << "/" << j+1;
          obj << " " << j+2 << "/" << j+2;// << "/" << j+2;
          obj << std::endl;
          j += 3;
        }
      }
      obj.close();
    }
    ++id;
  }
}
#endif





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
#ifndef IGNORE_COLOR
void
SurfaceReconstruction::ExportColoredPly
(const std::string& filename)
{
  const std::string& plyFilename(filename + ".ply");
  std::ofstream plyFile(plyFilename);
  
  plyFile << "ply" << '\n';
  plyFile << "format ascii 1.0" << '\n';
  plyFile << "element vertex " << this->fpTriangleMesh->GetNumOfVertices() << '\n';
  plyFile << "property float x" << '\n';
  plyFile << "property float y" << '\n';
  plyFile << "property float z" << '\n';
  plyFile << "property uchar diffuse_red" << '\n';
  plyFile << "property uchar diffuse_green" << '\n';
  plyFile << "property uchar diffuse_blue" << '\n';
  plyFile << "element face " << this->fpTriangleMesh->GetNumOfTriangles() << '\n';
  plyFile << "property list uchar int vertex_indices" << '\n';
  plyFile << "end_header" << '\n';
  
  std::map<const Vertex* const, unsigned long> vtxIdMap;
  unsigned long currVtxId = 0;
  
  BOOST_FOREACH(const Vertex& vtx, this->fpTriangleMesh->GetVertices())
  {
    plyFile << vtx.GetPosition().GetX() << ' ';
    plyFile << vtx.GetPosition().GetY() << ' ';
    plyFile << vtx.GetPosition().GetZ() << ' ';
    plyFile << static_cast<int>(255 * vtx.GetRed()) << ' ';
    plyFile << static_cast<int>(255 * vtx.GetGreen()) << ' ';
    plyFile << static_cast<int>(255 * vtx.GetBlue()) << ' ';
    plyFile << '\n';
    vtxIdMap[&vtx] = currVtxId++;
  }
  
  BOOST_FOREACH(const Triangle& tri, this->fpTriangleMesh->GetTriangles())
  {
    bool reorient = tri.NeedsReorient();
    plyFile << 3 << ' ';
    if (reorient)
    {
      plyFile << vtxIdMap[tri.GetV2()];
    }
    else
    {
      plyFile << vtxIdMap[tri.GetV0()];
    }
    plyFile << " ";
    plyFile << vtxIdMap[tri.GetV1()];
    plyFile << " ";
    
    if (reorient)
    {
      plyFile << vtxIdMap[tri.GetV0()];
    }
    else
    {
      plyFile << vtxIdMap[tri.GetV2()];
    }
    plyFile << ' ' << '\n';
  }
  plyFile << std::flush;
  plyFile.close();
}
#endif
