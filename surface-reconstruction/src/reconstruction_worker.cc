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
#include <cstdlib>

#include <iomanip>
#include <iostream>
#include <sstream>

#ifndef Q_MOC_RUN
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include <common/logging.h>

#include <Eigen/Dense>

#include "data_types.h"
#include "reconstruction_worker.h"
#include "surface_reconstruction.h"
#include "io/triangle_mesh_file.h"
#include "io/triangle_obj_file.h"



////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
ReconstructionWorker::ReconstructionWorker
(SurfaceReconstruction* pSurfaceReconstruction)
: fpSurfaceReconstruction(pSurfaceReconstruction)
, fRunning(false)
, fBeta(ProgramOptions::GetBeta())
, fEta(ProgramOptions::GetEta())
, fLambda(ProgramOptions::GetLambda())
, fVertexInactivityThreshold(ProgramOptions::GetVtxInactivityThreshold())
, fMaxEdgePenalty(ProgramOptions::GetAmax())
, fMaxLen2(ProgramOptions::GetMaxLen() * ProgramOptions::GetMaxLen())
, fDenseIterations(0)
{
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::Run
(boost::function<void()> finishedCallback,
 boost::function<void(const std::string&)> exportCallback)
{
  this->fRunning = true;
  const int lastFrame = ProgramOptions::GetLastFrame();

  const unsigned int killTime = ProgramOptions::GetKillTime();
  
#ifndef NO_FRAMELIST
  const std::string frameListFileName(ProgramOptions::GetFrameListFileFile());
  if (frameListFileName.size() != 0)
  {
    std::ifstream frameListFile(frameListFileName);
    std::string lineBuffer;
    bool readHeader = false;
    while (frameListFile.good())
    {
      std::getline(frameListFile, lineBuffer);
      if (!readHeader)
      {
        readHeader = true;
      }
      else
      {
        std::vector<std::string> tokenVector;
        boost::split(tokenVector, lineBuffer, boost::is_any_of(","));
        if (tokenVector.size() == 3)
        {
          this->fFrameIterations.push_back(std::pair<unsigned int, unsigned int>(atoi(tokenVector[1].c_str()), atoi(tokenVector[2].c_str())));
        }
      }
    }
    frameListFile.close();
  }
#endif
  
#ifndef NO_ITERATIVE_POINTCLOUDS
  const std::string filePrefix(ProgramOptions::GetFilePrefix());
  std::vector<std::string> iterativeFiles;
  boost::filesystem::directory_iterator dirIter(filePrefix);
  boost::filesystem::directory_iterator eod;
  BOOST_FOREACH(const boost::filesystem::path& p, std::make_pair(dirIter, eod))
  {
    if ((p.extension().compare(".cmvs") == 0) ||
        (p.extension().compare(".ply") == 0))
    {
      iterativeFiles.push_back(p.filename().string());
      std::cout << p.filename().string() << std::endl;
    }
  }

  std::vector<std::string>::iterator inputFileIter(iterativeFiles.begin());
  std::cout << *inputFileIter << std::endl;
  float pointVertexRatio = ProgramOptions::PointVertexRatio();
  bool lastRun = false;
#endif
  
  this->fStopWatch.Restart();
  while (this->fRunning)
  {
    if (this->fpSurfaceReconstruction->GetPointCloud()->IsAvailable() ||
        this->fpSurfaceReconstruction->GetDensePointCloud()->IsAvailable())
    {
      this->fpSurfaceReconstruction->IncrementIterationsDone();
      
      // reconstruction step
      if (this->fpSurfaceReconstruction != NULL)
      {
        this->DoSingleReconstructionStep();
      }
      
#ifndef NO_FRAMELIST
      if (this->fFrameIterations.size() != 0 &&
          this->fFrameIterations.front().second == this->fpSurfaceReconstruction->GetIterationsDone())
      {
        const unsigned int frame = this->fFrameIterations.front().first;
        
        if (frame > lastFrame)
        {
          const std::string filename(ProgramOptions::GetMeshFile());
          
          std::stringstream frameStream;
          frameStream << std::setw(5) << std::setfill('0') << frame;
          
          namespace bfs = boost::filesystem;
          const bfs::path& filePath(filename);
          if (!bfs::exists(filePath.parent_path()))
          {
            bfs::create_directories(filePath.parent_path());
          }
          
          bfs::path fileStem(filePath.stem());
#ifndef NO_ITERATIVE_POINTCLOUDS
          fileStem += "--i";
          fileStem += (*inputFileIter)[0];
          fileStem += (*inputFileIter)[1];
          fileStem += (*inputFileIter)[2];
          fileStem += (*inputFileIter)[3];
#endif
          fileStem += "--f";
          fileStem += frameStream.str().c_str();
          fileStem += ".obj";
          
          bfs::path objFilePath(filePath.parent_path());
          objFilePath /= fileStem;

          this->fpSurfaceReconstruction->ExportObj(objFilePath.string());
        }
        this->fFrameIterations.pop_front();
        std::cout << "Next export at iteration: " << this->fFrameIterations.front().second << std::endl;
      }
#endif
      
      // node addition and removel
      if (this->fpSurfaceReconstruction->GetIterationsDone() %
            this->fLambda == 0)
      {
        this->DoEdgeSplit();
        if ((this->fpSurfaceReconstruction->GetTriangleMesh()->GetNumOfVertices() >
             (this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints() +
              this->fpSurfaceReconstruction->GetDensePointCloud()->GetNumOfPoints()) / 4) ||
            this->fpSurfaceReconstruction->GetTriangleMesh()->GetNumOfVertices() > 1000)
        {
          this->DoVertexRemoval();
        }
      }
      
      
      
      // print timings
      if (this->fpSurfaceReconstruction->GetIterationsDone() % 100000 == 0)
      {
        //this->fStopWatch.Stop();
        this->PrintStatistics();
        if (killTime > 0u &&
            this->fStopWatch.ElapsedMilliseconds() > killTime * 1000)
        {
          LOG_INFO("Killed");
          return;
        }
        //this->fStopWatch.Start();
      }
    
      // stop reconstructing if finished
      if (this->fpSurfaceReconstruction->GetTargetNumVertices() <= this->fpSurfaceReconstruction->GetTriangleMesh()->GetNumOfVertices())
      {
        this->fRunning = false;
      }
      
#ifndef NO_ITERATIVE_POINTCLOUDS
      if ((!lastRun &&
           pointVertexRatio > 0.0 &&
           pointVertexRatio * this->fpSurfaceReconstruction->GetTriangleMesh()->GetNumOfVertices() >=
           this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints()))
      {
        this->fStopWatch.Stop();
        //this->fRunning = false;
        this->PrintStatistics();

        if (inputFileIter != iterativeFiles.end())
        {
          ++inputFileIter;
        }
        
        if (inputFileIter == iterativeFiles.end())
        {
          lastRun = true;
          this->fRunning = false;
        }
        else
        {
          std::cout << *inputFileIter << std::endl;
          this->fpSurfaceReconstruction->LoadInputFile(filePrefix + "/" + (*inputFileIter));
          this->fStopWatch.Start();
        }
      }
      if (lastRun &&
          2 * this->fpSurfaceReconstruction->GetTriangleMesh()->GetNumOfVertices() >=
          this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints())
      {
        this->fStopWatch.Stop();
        this->fRunning = false;
      }
#endif
    }
  };
  
  LOG_INFO("Reconstruction time: " << this->fStopWatch.ElapsedMilliseconds() * 0.001f);
  this->fpSurfaceReconstruction->GetTriangleMesh()->Info();
  this->PrintStatistics();
  
  std::cout << "# beta: " << this->GetBeta() << std::endl;
  std::cout << "# eta: " << this->GetEta() << std::endl;
  std::cout << "# lambda: " << this->GetLambda() << std::endl;
  std::cout << "# vtx inact. thd.: " << this->GetVertexInactivityThreshold() << std::endl;
  std::cout << "# max edge penalty: " << this->GetMaxEdgePenalty() << std::endl;
  std::cout << "# max len^2: " << this->GetMaxLen2() << std::endl;
  const AABB_T<Flt> bbPoints(this->fpSurfaceReconstruction->GetPointCloud()->GetBoundingBox());
  const float diag = (bbPoints.GetMaxCorner() - bbPoints.GetMinCorner()).GetLength();
  std::cout << "# bb diag (points): " << diag << std::endl;
  
  this->fpSurfaceReconstruction->ExportObj(ProgramOptions::GetMeshFile());

  // call finishedCallback
  if (finishedCallback != NULL)
  {
    finishedCallback();
  }
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::DoSingleReconstructionStep
()
{
  // lock mesh mutex
  this->fpSurfaceReconstruction->LockMesh();
  this->fpSurfaceReconstruction->LockPoints();
  
  const PointCloud::SharedPtr pPointCloud(
    this->fpSurfaceReconstruction->GetPointCloud());
  
  // get signal from point cloud
  const Point* const samplePoint = pPointCloud->GetNext();
  const Position samplePosition(samplePoint->GetPosition());
  
  // obtain mesh pointer
  TriangleMesh::SharedPtr const pTriangleMesh = 
    this->fpSurfaceReconstruction->GetTriangleMesh();
  
  // store edges to repair on operations that potentially reduce edge valence
  std::vector<Edge*> edgesToRepair;
  edgesToRepair.reserve(48);

  
  // find the two closest neurons
  Vertex* closestVertices[2] = { NULL, NULL };
  pTriangleMesh->FindClosestVertices(samplePosition, closestVertices);
  pTriangleMesh->IncrementBmuCount(closestVertices[0]);
  
  assert(closestVertices[0] != NULL);
  assert(closestVertices[1] != NULL);
  assert(closestVertices[0] != closestVertices[1]);
  assert(closestVertices[1] != closestVertices[0]);
  
  Edge* pInterconnectingEdge = this->FindInterconnectinEdge(closestVertices[0],
                                                            closestVertices[1]);
  
  // Update BMU
  this->UpdateVertex(closestVertices[0],
                     this->fBeta,
                     samplePoint,
                     pTriangleMesh.get());
  Triangle* const pAdjTri0 =
    pInterconnectingEdge == NULL ? NULL : pInterconnectingEdge->GetT0();
  Triangle* const pAdjTri1 =
    pInterconnectingEdge == NULL ? NULL : pInterconnectingEdge->GetT1();
  
  this->UpdateNeighbours(closestVertices[0],
                         this->fEta,
                         samplePoint,
                         pTriangleMesh.get());

  Triangle* const pT0 = pAdjTri0;
  Triangle* const pT1 = pAdjTri1;
  
  const Vertex* const pVL =
  pT0 != NULL ? pT0->GetOther(pInterconnectingEdge->GetV0(),
                              pInterconnectingEdge->GetV1())
  : NULL;
  const Vertex* const pVR =
  pT1 != NULL ? pT1->GetOther(pInterconnectingEdge->GetV0(),
                              pInterconnectingEdge->GetV1())
  : NULL;
  
  const Flt dist2ToVL =
  pVL != NULL ? (samplePosition - pVL->GetPosition()).GetSquaredLength()
  : std::numeric_limits<Flt>::infinity();
  const Flt dist2ToVR =
  pVR != NULL ? (samplePosition - pVR->GetPosition()).GetSquaredLength()
  : std::numeric_limits<Flt>::infinity();
  
  
  
  if (pAdjTri0 != NULL && dist2ToVL <= dist2ToVR)
  {
    this->FitBarycentric(pAdjTri0, this->fBeta, samplePoint, pTriangleMesh.get());
  }
  else if (pAdjTri1 != NULL && dist2ToVL > dist2ToVR)
  {
    this->FitBarycentric(pAdjTri1, this->fBeta, samplePoint, pTriangleMesh.get());
  }
  
  
  
  
  
  if ((pVL != NULL) && (dist2ToVL <= dist2ToVR))
  {
    pT0->DecAge();
    if (pT1 != NULL)
    {
      pT1->IncAge();
      if (pT1->GetAge() > 20)
      {
        pTriangleMesh->DeleteTriangle(pT1);
      }
    }
  }
  else if ((pVR != NULL) && (dist2ToVL > dist2ToVR))
  {
    pT1->DecAge();
    if (pT0 != NULL)
    {
      pT0->IncAge();
    }
    if (pT0->GetAge() > 20)
    {
      pTriangleMesh->DeleteTriangle(pT0);
    }
  }


  this->DoEdgeAgeingAndDeletion(closestVertices[0],
                                pInterconnectingEdge,
                                pTriangleMesh.get(),
                                &edgesToRepair);
  
  this->UpdateVertexActivity(closestVertices[0], pTriangleMesh.get());
  
  // Try to connect the neurons with an edge
  // If it already exists, use it
  // Otherwise create new edge
  
  std::vector<Vertex*> commonNeighbours;
  pTriangleMesh->FindCommonNeighbours(closestVertices[0],
                                      closestVertices[1],
                                      &commonNeighbours);
  
  if (commonNeighbours.size() > 2)
  {
    Vertex* pCNuse[2] = {NULL, NULL};
    Activity act[2] = {0, 0};
    BOOST_FOREACH(Vertex* pCN, commonNeighbours)
    {
      if (pCN->GetActivity() >= act[0])
      {
        act[1] = act[0];
        pCNuse[1] = pCNuse[0];
        act[0] = pCN->GetActivity();
        pCNuse[0] = pCN;
      }
      else if (pCN->GetActivity() >= act[1])
      {
        act[1] = pCN->GetActivity();
        pCNuse[1] = pCN;
      }
    }
    commonNeighbours.clear();
    commonNeighbours.push_back(pCNuse[0]);
    commonNeighbours.push_back(pCNuse[1]);
  }

  if (pInterconnectingEdge == NULL)
  {
    pInterconnectingEdge = this->CreateOrFlipInterconnection(commonNeighbours,
                                                             closestVertices,
                                                             pTriangleMesh);
  }
  
  if (pInterconnectingEdge != NULL)
  {
    // select those common neighbours that lead to least roughness
    if (commonNeighbours.size() > 2)
    {
      pTriangleMesh->SelectTwoLeastRoughNeighbours(closestVertices[0],
                                                   closestVertices[1],
                                                   &commonNeighbours);
    }
    
    // create triangles for every common neighbour
    // store other vertices of existing triangles
    this->CreateTriangles(pInterconnectingEdge,
                          commonNeighbours,
                          closestVertices,
                          pTriangleMesh,
                          &edgesToRepair);
    
    // check if interconnecting edge is border on quadrangular hole
    if ((pInterconnectingEdge->GetValence() != 2))
    {
      Vertex* const pV0 = pInterconnectingEdge->GetV0();
      Vertex* const pV1 = pInterconnectingEdge->GetV1();
      bool a = pV0->GetValence() < pV1->GetValence();
      
      FourLoop fourLoop(a ? pV0 : pV1,
                        NULL,
                        NULL,
                        a ? pV1 : pV0,
                        NULL,
                        NULL,
                        NULL,
                        pInterconnectingEdge);
      this->QuadLoop2(&fourLoop, 0);

      this->QuadAtInterconnection(pInterconnectingEdge,
                                  pTriangleMesh,
                                  &edgesToRepair);
    }
    if (pInterconnectingEdge->GetValence() != 2)
    {
      edgesToRepair.push_back(pInterconnectingEdge);
    }
  }

  if (edgesToRepair.size() > 0)
  {
    BOOST_FOREACH(Edge* pEdge, edgesToRepair)
    {
      if (pEdge->GetValence() < 2)
      {
        Vertex* const pV0 = pEdge->GetV0();
        Vertex* const pV1 = pEdge->GetV1();
        if (pV0 != NULL && pV1 != NULL)
        {
          bool a = pV0->GetValence() < pV1->GetValence();
          
          FourLoop fourLoop(a ? pV0 : pV1,
                            NULL,
                            NULL,
                            a ? pV1 : pV0,
                            NULL,
                            NULL,
                            NULL,
                            pEdge);
          this->QuadLoop2(&fourLoop, 0);
        }
      }
    }
    
    pTriangleMesh->RepairEdges(edgesToRepair);
  }
  
  // unlock mesh mutex
  this->fpSurfaceReconstruction->UnlockPoints();
  this->fpSurfaceReconstruction->UnlockMesh();
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::UpdateVertex
(Vertex* pVertex,
 Flt learningRate,
 const Point* pSamplePoint,
 TriangleMesh* pTriMesh,
 bool isNeighbour)
{
  const Flt invertedRate = sc<Flt>(1.0) - learningRate;
  const Vector movement = pSamplePoint->GetPosition() - pVertex->GetPosition();
  
  pTriMesh->MoveVertex(pVertex, learningRate * movement);
  pVertex->SetNormal(
    invertedRate * pVertex->GetNormal() +
    learningRate * pSamplePoint->GetNormal());
  
  // learn color
#ifndef IGNORE_COLOUR
  if (!isNeighbour)
  {
    pVertex->SetRed(  invertedRate * pVertex->GetRed() +
                      learningRate * pSamplePoint->GetRed());
    pVertex->SetGreen(invertedRate * pVertex->GetGreen() +
                      learningRate * pSamplePoint->GetGreen());
    pVertex->SetBlue( invertedRate * pVertex->GetBlue() +
                      learningRate * pSamplePoint->GetBlue());
  }
#endif
  
#ifndef NO_TEXTURES
  // learn texturing
  if (!isNeighbour)
  {
    BOOST_FOREACH(TextureCoordinate& texCoord,
                  pVertex->GetTextureCoordinates())
    {
      texCoord.SetConfidence(sc<Flt>(0.99) * texCoord.GetConfidence());
    }
  }
  TextureCoordinate::AdaptTo(pSamplePoint->GetTextureCoordinates(),
                             learningRate,
                             pVertex->GetTextureCoordinates());
  pVertex->IncAdaptCount();
#endif
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::UpdateNeighbours
(Vertex* pAroundVertex,
 Flt learningRate,
 const Point* pSamplePoint,
 TriangleMesh* pTriMesh)
{
  BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pAroundVertex))
  {
    assert(pAdjEdge->GetIsActive());
    Vertex* const pNeighbour = pAdjEdge->GetOther(pAroundVertex);
    
    const bool isNeighbour = true;
    this->UpdateVertex(pNeighbour,
                       learningRate,
                       pSamplePoint,
                       pTriMesh,
                       isNeighbour);
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
float
ReconstructionWorker::MaxEdgeDistance
(Triangle* pTriangle,
 const Point* pSamplePoint)
{
  const Vertex* pV0 = pTriangle->GetV0();
  const Vertex* pV1 = pTriangle->GetV1();
  const Vertex* pV2 = pTriangle->GetV2();
  
  const Position& posV0 = pV0->GetPosition();
  const Position& posV1 = pV1->GetPosition();
  const Position& posV2 = pV2->GetPosition();
  
  const Vector v0v1 = normalize(posV1 - posV0);
  const Vector v1v2 = normalize(posV2 - posV1);
  const Vector v2v0 = normalize(posV0 - posV2);
  
  const Eigen::Vector3f v0(posV0.GetX(), posV0.GetY(), posV0.GetZ());
  const Eigen::Vector3f v1(posV1.GetX(), posV1.GetY(), posV1.GetZ());
  const Eigen::Vector3f v2(posV2.GetX(), posV2.GetY(), posV2.GetZ());
  
  const Position& posSample(pSamplePoint->GetPosition());
  const Eigen::Vector3f p(posSample.GetX(), posSample.GetY(), posSample.GetZ());
  
  const Eigen::Vector3f normal((v1-v0).cross(v2-v0));
  const float sqLenNormal = normal.squaredNorm();
  const float lenNormal = sqrt(sqLenNormal);
  const Eigen::Vector3f normedNormal(normal / lenNormal);
  
  const Eigen::Vector3f delta(p-v0);
  const Eigen::Vector3f perp(normedNormal.dot(delta) * normedNormal);
  const Eigen::Vector3f inPlane(delta - perp);
  
  const Eigen::Vector3f pointInPlane(p - perp);
  
  const float alpha = (v1-v0).cross(pointInPlane - v0).dot(normal) / sqLenNormal;
  const float gamma = (pointInPlane - v0).cross(v2-v0).dot(normal) / sqLenNormal;
  const float beta = 1.0f - alpha - gamma;
  //const float beta = (pointInPlane - v2).cross(v1-v2).dot(normal) / sqLenNormal;
  
  const float xDist = lenNormal * alpha / (v1-v0).norm();
  const float yDist = lenNormal * beta  / (v2-v1).norm();
  const float zDist = lenNormal * gamma / (v2-v0).norm();
  
  
  return -std::min(0.0f, std::min(xDist, std::min(yDist, zDist)));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::FitBarycentric
(Triangle* pTriangle,
 Flt rate,
 const Point* pSamplePoint,
 TriangleMesh* pTriMesh)
{
  const Vertex* pV0 = pTriangle->GetV0();
  const Vertex* pV1 = pTriangle->GetV1();
  const Vertex* pV2 = pTriangle->GetV2();
  
  const Position& posV0 = pV0->GetPosition();
  const Position& posV1 = pV1->GetPosition();
  const Position& posV2 = pV2->GetPosition();
  
  const Vector v0v1 = normalize(posV1 - posV0);
  const Vector v1v2 = normalize(posV2 - posV1);
  const Vector v2v0 = normalize(posV0 - posV2);
  
  const Eigen::Vector3f v0(posV0.GetX(), posV0.GetY(), posV0.GetZ());
  const Eigen::Vector3f v1(posV1.GetX(), posV1.GetY(), posV1.GetZ());
  const Eigen::Vector3f v2(posV2.GetX(), posV2.GetY(), posV2.GetZ());
  
  const Position& posSample(pSamplePoint->GetPosition());
  const Eigen::Vector3f p(posSample.GetX(), posSample.GetY(), posSample.GetZ());
  
  const Eigen::Vector3f normal((v1-v0).cross(v2-v0));
  const float sqLenNormal = normal.squaredNorm();
  const float lenNormal = sqrt(sqLenNormal);
  const Eigen::Vector3f normedNormal(normal / lenNormal);
  
  const Eigen::Vector3f delta(p-v0);
  const Eigen::Vector3f perp(normedNormal.dot(delta) * normedNormal);
  const Eigen::Vector3f inPlane(delta - perp);
  
  const Eigen::Vector3f pointInPlane(p - perp);
  
  const float alpha = (v1-v0).cross(pointInPlane - v0).dot(normal) / sqLenNormal;
  const float gamma = (pointInPlane - v0).cross(v2-v0).dot(normal) / sqLenNormal;
  const float beta = 1.0f - alpha - gamma;
  //const float beta = (pointInPlane - v2).cross(v1-v2).dot(normal) / sqLenNormal;
  
  const float xDist= lenNormal * alpha / (v1-v0).norm();
  const float yDist = lenNormal * beta / (v2-v1).norm();
  const float zDist = lenNormal * gamma / (v2-v0).norm();
  
  if (xDist < 0.0)
  {
    pTriMesh->MoveVertex(pTriangle->GetV0(), -rate * xDist * v2v0);
    pTriMesh->MoveVertex(pTriangle->GetV1(),  rate * xDist * v1v2);
  }
  if (yDist < 0.0)
  {
    pTriMesh->MoveVertex(pTriangle->GetV1(), -rate * yDist * v0v1);
    pTriMesh->MoveVertex(pTriangle->GetV2(),  rate * yDist * v2v0);
  }
  if (zDist < 0.0)
  {
    pTriMesh->MoveVertex(pTriangle->GetV2(), -rate * zDist * v1v2);
    pTriMesh->MoveVertex(pTriangle->GetV0(),  rate * zDist * v0v1);
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Edge*
ReconstructionWorker::FindInterconnectinEdge
(Vertex* pVtxA,
 Vertex* pVtxB)
{
  BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pVtxA))
  {
    if (pAdjEdge->Connects(pVtxA, pVtxB))
    {
      return pAdjEdge;
    }
  }
  return NULL;
}



////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::DoEdgeAgeingAndDeletion
(Vertex *pBmu,
 Edge *pInterconnectingEdge,
 TriangleMesh* pTriangleMesh,
 std::vector<Edge *> *pEdgesToRepair)
{
  // store avg edge length of tri mesh
  const float avgEdgeLength =
  this->fpSurfaceReconstruction->GetTriangleMesh()->GetAverageEdgeLength();
  
  // reserve some space to store too old edges
  std::vector<Edge*> edgesToDelete;
  edgesToDelete.reserve(12);
  
  BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pBmu))
  {
    // learn avg edge length
    const float edgeLength = pAdjEdge->GetSquaredLength();
    this->fpSurfaceReconstruction->GetTriangleMesh()->SetAverageEdgeLength(
      avgEdgeLength + 0.1f * (edgeLength - avgEdgeLength));
    
    // check if this is the inteconnecting edge
    if (pAdjEdge == pInterconnectingEdge)
    {
      pAdjEdge->SetAge(static_cast<Edge::Age>(0));
    }
    
    // edge aging
    assert(pAdjEdge->GetIsActive());
    if (pAdjEdge->IsFaceless() ||
        this->ThalesCircleCheck(pAdjEdge, pBmu))
    {
      pAdjEdge->IncrementAge();
    }
    
    // +1 is used to compensate aging of interconnecting edge
    if (pAdjEdge->IsOlderThan(this->fMaxEdgePenalty + 1))
    {
      edgesToDelete.push_back(pAdjEdge);
    }
  }
  
  BOOST_FOREACH(Edge* pEdge, edgesToDelete)
  {
    pTriangleMesh->DeleteEdge(pEdge, true, pEdgesToRepair);
  }
}



////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Edge*
ReconstructionWorker::ProcessEdgeStar
(Vertex* pClosestVertices[2],
 const Point* pSamplePoint,
 TriangleMesh* pTriangleMesh,
 std::vector<Edge*>* pEdgesToRepair )
{
  Edge* pInterconnectingEdge = NULL;
  
  // store avg edge length of tri mesh
  const float avgEdgeLength =
  this->fpSurfaceReconstruction->GetTriangleMesh()->GetAverageEdgeLength();
  
  // reserve some space to store too old edges
  std::vector<Edge*> edgesToDelete;
  edgesToDelete.reserve(12);
  
  BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pClosestVertices[0]))
  {
    assert(pAdjEdge->GetIsActive());
    Vertex* const pNeighbour = pAdjEdge->GetOther(pClosestVertices[0]);
    
    const bool isNeighbour = true;
    this->UpdateVertex(pNeighbour,
                       this->fEta,
                       pSamplePoint,
                       pTriangleMesh,
                       isNeighbour);
    
    // learn avg edge length
    const float edgeLength = pAdjEdge->GetSquaredLength();
    this->fpSurfaceReconstruction->GetTriangleMesh()->SetAverageEdgeLength(
                                                                           avgEdgeLength + 0.1f * (edgeLength - avgEdgeLength));
    
    // check if this is the inteconnecting edge
    if (pAdjEdge->Connects(pClosestVertices[0], pClosestVertices[1]))
    {
      pInterconnectingEdge = pAdjEdge;
      pInterconnectingEdge->SetAge(static_cast<Edge::Age>(0));
    }
    
    // edge aging
    assert(pAdjEdge->GetIsActive());
    if (pAdjEdge->IsFaceless() ||
        pAdjEdge->IsLongerThan(this->fMaxLen2 * avgEdgeLength) ||
        this->ThalesCircleCheck(pAdjEdge, pClosestVertices[0]))
    {
      pAdjEdge->IncrementAge();
    }
    
    // +1 is used to compensate aging of interconnecting edge
    if (pAdjEdge->IsOlderThan(this->fMaxEdgePenalty + 1))
    {
      edgesToDelete.push_back(pAdjEdge);
    }
  }
  
  BOOST_FOREACH(Edge* pEdge, edgesToDelete)
  {
    pTriangleMesh->DeleteEdge(pEdge, true, pEdgesToRepair);
  }
  
  return pInterconnectingEdge;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::UpdateVertexActivity
(Vertex *pVtx,
 TriangleMesh* pTriangleMesh)
{
  // Increment best match's activity
  pTriangleMesh->IncrementVertexActivity(
    pVtx,
    this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints() +
    this->fpSurfaceReconstruction->GetDensePointCloud()->GetNumOfPoints());
  pTriangleMesh->ActivateVertex(
    pVtx,
    this->fpSurfaceReconstruction->GetIterationsDone(),
    this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints() +
    this->fpSurfaceReconstruction->GetDensePointCloud()->GetNumOfPoints());
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::DoEdgeSplit
()
{
  // lock mesh mutex
  this->fpSurfaceReconstruction->LockMesh();
  
  // obtain mesh pointer
  TriangleMesh::SharedPtr const pTriangleMesh = 
    this->fpSurfaceReconstruction->GetTriangleMesh();
  
  // find most active vertex
  Vertex* pMostActiveVertex = pTriangleMesh->FindMostActiveVertex(
    this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints() +
    this->fpSurfaceReconstruction->GetDensePointCloud()->GetNumOfPoints());
  
  if (pMostActiveVertex == NULL)
  {
    this->fpSurfaceReconstruction->UnlockMesh();
    return;
  }
  
  
  if (pMostActiveVertex->GetValence() > 0)
  {
    // find longest edge adj. to most active vertex
    Flt longestSquaredLength = sc<Flt>(0.0);
    Edge* pLongestEdge = NULL;
    BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pMostActiveVertex))
    {
      assert(pAdjEdge->GetIsActive());
      Flt currSquaredLength = pAdjEdge->GetSquaredLength();
      
      if (currSquaredLength >= longestSquaredLength)
      {
        longestSquaredLength = currSquaredLength;
        pLongestEdge = pAdjEdge;
      }
    }
    
    Vertex* pVA = pLongestEdge->GetV0();
    Vertex* pVB = pLongestEdge->GetV1();
    
    const Position posA(pVA->GetPosition());
    const Position posB(pVB->GetPosition());
    const Position posC(sc<Flt>(0.5) * (posA + posB));
    Vector normalC(pVA->GetNormal() + pVB->GetNormal());
    
    Vertex* thirdVerticesOfAdjTris[2] = { NULL, NULL };
    if (pLongestEdge->GetT0() != NULL)
    {
      thirdVerticesOfAdjTris[0] = pLongestEdge->GetT0()->GetOther(pVA, pVB);
    }
    if (pLongestEdge->GetT1() != NULL)
    {
      thirdVerticesOfAdjTris[1] = pLongestEdge->GetT1()->GetOther(pVA, pVB);
    }
    
    pTriangleMesh->DeleteEdge(pLongestEdge, false);
    
    Vertex* pVC = pTriangleMesh->NewVertex(posC);
    pVC->SetNormal(normalC);
#ifndef IGNORE_COLOUR
    pVC->SetRed(static_cast<Point::ColorComponent>(0.5) * (pVA->GetRed() +
                                                           pVB->GetRed()));
    pVC->SetGreen(static_cast<Point::ColorComponent>(0.5) * (pVA->GetGreen() +
                                                             pVB->GetGreen()));
    pVC->SetBlue(static_cast<Point::ColorComponent>(0.5) * (pVA->GetBlue() +
                                                            pVB->GetBlue()));
#endif
    
#ifndef NO_TEXTURES
    TextureCoordinate::SetAverage(pVA->GetTextureCoordinates(),
                                  pVB->GetTextureCoordinates(),
                                  pVC->GetTextureCoordinates());
    int avgAdaptCount =
      sc<Flt>(pVA->AdaptCount() + pVB->AdaptCount()) / sc<Flt>(2.0) + sc<Flt>(0.5);
    pVC->SetAdaptCount(avgAdaptCount);
#endif
    
    Edge* const pEdgeAC = pTriangleMesh->NewEdge(pVA, pVC);
    Edge* const pEdgeCB = pTriangleMesh->NewEdge(pVC, pVB);
    
    pEdgeAC->SetAge(Edge::Age(static_cast<Edge::Age>(0)));
    pEdgeCB->SetAge(Edge::Age(static_cast<Edge::Age>(0)));
    
    for (unsigned int i = 0; i < 2; ++i)
    {
      Vertex* const pVN = thirdVerticesOfAdjTris[i];
      if (pVN != NULL)
      {
        Edge* const pEdgeNC = pTriangleMesh->NewEdge(pVN, pVC);
        pEdgeNC->SetAge(Edge::Age(static_cast<Edge::Age>(0)));
        
        pTriangleMesh->NewTriangle(pVA, pVN, pVC);
        pTriangleMesh->NewTriangle(pVB, pVC, pVN);
      }
    }
    
    Vertex* const pVL = thirdVerticesOfAdjTris[0];
    Vertex* const pVR = thirdVerticesOfAdjTris[1];
    
    pTriangleMesh->DistributeBmuCountOnSplit(pVA, pVB, pVL, pVR, pVC);
    pTriangleMesh->MakeVertexActivityNeutral(pVA);
    pTriangleMesh->MakeVertexActivityNeutral(pVB);
    
    pTriangleMesh->ActivateVertex(
      pVA, this->fpSurfaceReconstruction->GetIterationsDone(),
      (this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints() +
       this->fpSurfaceReconstruction->GetDensePointCloud()->GetNumOfPoints()));
    pTriangleMesh->ActivateVertex(
      pVB, this->fpSurfaceReconstruction->GetIterationsDone(),
      (this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints() +
       this->fpSurfaceReconstruction->GetDensePointCloud()->GetNumOfPoints()));
    pTriangleMesh->ActivateVertex(
      pVC, this->fpSurfaceReconstruction->GetIterationsDone(),
      (this->fpSurfaceReconstruction->GetPointCloud()->GetNumOfPoints() +
       this->fpSurfaceReconstruction->GetDensePointCloud()->GetNumOfPoints()));
  }
  
  // unlock mesh mutex
  this->fpSurfaceReconstruction->UnlockMesh();
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::DoVertexRemoval
()
{
  // lock mesh mutex
  this->fpSurfaceReconstruction->LockMesh();
  
  // obtain mesh pointer
  TriangleMesh::SharedPtr const pTriangleMesh = 
    this->fpSurfaceReconstruction->GetTriangleMesh();
  
  // select vertices that are inactive
  std::vector<Vertex*> verticesToRemove;
  pTriangleMesh->FindInactiveVertices(
    this->fpSurfaceReconstruction->GetIterationsDone(),
    this->fVertexInactivityThreshold,
    &verticesToRemove);
  
  // remove inactive vertices by edge collapse
  BOOST_FOREACH(Vertex* pVtx, verticesToRemove)
  {
    // find edge (adj. to pVtx) to collapse
    Edge* pEdgeToCollapse = this->FindEdgeToCollapse(pVtx, pTriangleMesh.get());
    
    // collapse edge, i.e., delete it
    if (pEdgeToCollapse != NULL)
    {
      // rename/get pointers to vertices at edge
      Vertex* const pVtxSource = pVtx;
      Vertex* const pVtxTarget = pEdgeToCollapse->GetOther(pVtxSource);
      
      // get pointers to adjacent triangles
      Triangle* const pTriLeft = pEdgeToCollapse->GetT0();
      Triangle* const pTriRight = pEdgeToCollapse->GetT1();
      
      // get pointers to left and right vertices
      Vertex* const pVtxLeft = (pTriLeft != NULL) ? 
        pTriLeft->GetOther(pVtxSource, pVtxTarget) : NULL;
      Vertex* const pVtxRight = (pTriRight != NULL) ? 
        pTriRight->GetOther(pVtxSource, pVtxTarget) : NULL;
     
      pTriangleMesh->DistributeBmuCountOnCollapse(pVtxSource, pVtxTarget,
                                                  pVtxLeft, pVtxRight);
      
      // get pointers to edges connecting source/target -- left/right
      Edge* const pEdgeSourceLeft = (pVtxLeft != NULL) ?
        pTriangleMesh->GetEdge(pVtxSource, pVtxLeft) : NULL;
      Edge* const pEdgeTargetLeft = (pVtxLeft != NULL) ?
        pTriangleMesh->GetEdge(pVtxTarget, pVtxLeft) : NULL;
      Edge* const pEdgeSourceRight = (pVtxRight != NULL) ?
        pTriangleMesh->GetEdge(pVtxSource, pVtxRight) : NULL;
      Edge* const pEdgeTargetRight = (pVtxRight != NULL) ?
        pTriangleMesh->GetEdge(pVtxTarget, pVtxRight) : NULL;
      
      // delete edge including their adjacent triangles
      pTriangleMesh->DeleteEdge(pEdgeToCollapse, false);
      
      // handle all the other edges/triangles
      if (pEdgeSourceLeft != NULL)
      {
        if (pEdgeSourceLeft->GetValence() < 2)
        {
          Triangle* pAdjTriangle = pEdgeSourceLeft->GetT0();
          if (pAdjTriangle != NULL)
          {
            pAdjTriangle->ReplaceEdge(pEdgeSourceLeft, pEdgeTargetLeft);
          }
          if (pEdgeSourceLeft->GetValence() == 0)
          {
            pTriangleMesh->DeleteEdge(pEdgeSourceLeft, false);
          }
        }
      }
      if (pEdgeSourceRight != NULL)
      {
        if (pEdgeSourceRight->GetValence() < 2)
        {
          Triangle* pAdjTriangle = pEdgeSourceRight->GetT0();
          if (pAdjTriangle != NULL)
          {
            pAdjTriangle->ReplaceEdge(pEdgeSourceRight, pEdgeTargetRight);
          }
          if (pEdgeSourceRight->GetValence() == 0)
          {
            pTriangleMesh->DeleteEdge(pEdgeSourceRight, false);
          }
        }
      }
      
      std::vector<Edge*> edgesLeavingSource(Adjacent<Edge>::Begin(pVtxSource),
                                            Adjacent<Edge>::End(pVtxSource));
      BOOST_FOREACH(Edge* pAdjEdge, edgesLeavingSource)
      {
        pAdjEdge->ReplaceVertex(pVtxSource, pVtxTarget);
        if (pAdjEdge->GetT0() != NULL)
        {
          pAdjEdge->GetT0()->DetermineVertices();
        }
        if (pAdjEdge->GetT1() != NULL)
        {
          pAdjEdge->GetT1()->DetermineVertices();
        }
      }
      
      if (pVtxSource->GetValence() == 0)
      {
        pTriangleMesh->DeleteVertex(pVtxSource);
        if ((pVtxLeft != NULL) && (pVtxLeft->GetValence() == 0))
        {
          pTriangleMesh->DeleteVertex(pVtxLeft);
        }
        if ((pVtxRight != NULL) && (pVtxRight->GetValence() == 0))
        {
          pTriangleMesh->DeleteVertex(pVtxRight);
        }
      }
    }// if (pEdgeToCollapse != NULL)
  } // BOOST_FOREACH(Vertex* pVtx, verticesToRemove)
  
  // unlock mesh mutex
  this->fpSurfaceReconstruction->UnlockMesh();
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Edge*
ReconstructionWorker::FindEdgeToCollapse
(Vertex* pVtxSrc,
 TriangleMesh* pTriangleMesh)
{
  Edge* pEdgeToCollapse = NULL;
  unsigned int bestCollapseError = std::numeric_limits<unsigned int>::max();
  BOOST_FOREACH(Edge *pAdjEdge, Adjacent<Edge>::Around(pVtxSrc))
  {
    assert(pAdjEdge->GetIsActive());
    // store edge valence
    const unsigned int adjEdgeValence = pAdjEdge->GetValence();
    
    // only valid if pAdjEdge->pV0 and pAdjEdge->pV1
    // share max 1 (border) or 2 (internal) common neighbours (see Hoppe)
    std::vector<Vertex*> commonNeighbours;
    pTriangleMesh->FindCommonNeighbours(pAdjEdge->GetV0(),
                                        pAdjEdge->GetV1(),
                                        &commonNeighbours);
    if (((adjEdgeValence == 0) && (commonNeighbours.size() > 0)) ||
        ((adjEdgeValence == 1) && (commonNeighbours.size() > 1)) ||
        ((adjEdgeValence == 2) && (commonNeighbours.size() > 2)))
    {
      continue;
    }
    
    // check if collappse error is lower as previous one
    unsigned int currCollapseError =
    (pAdjEdge->GetValence() == 0) ? 0 :
    pTriangleMesh->CalculateCollapseError(pAdjEdge,
                                          commonNeighbours.front(),
                                          commonNeighbours.back());
    if (currCollapseError < bestCollapseError)
    {
      bestCollapseError = currCollapseError;
      pEdgeToCollapse = pAdjEdge;
    }
  }
  return pEdgeToCollapse;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
bool
ReconstructionWorker::ThalesCircleCheck
(const Edge* pEdge,
 const Vertex* pAroundVertex)
{
  const Position thalesCenter(
    sc<Flt>(0.5) * (pEdge->GetV0()->GetPosition() +
                    pEdge->GetV1()->GetPosition()));
  const Flt thalesRadius2 = sc<Flt>(0.25) * pEdge->GetSquaredLength();
  BOOST_FOREACH(Edge* pOtherEdge, Adjacent<Edge>::Around(pAroundVertex))
  {
    if (pOtherEdge != pEdge)
    {
      const Position otherPos(
        pOtherEdge->GetOther(pAroundVertex)->GetPosition());
      if ((otherPos - thalesCenter).GetSquaredLength() < thalesRadius2)
      {
        return true;
      }
    }
  }
  return false;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
Edge*
ReconstructionWorker::CreateOrFlipInterconnection
(const std::vector<Vertex*>& commonNeighbours,
 Vertex* pClosestVertices[2],
 TriangleMesh::SharedPtr pTriangleMesh)
{
  Edge* pInterconnectingEdge = NULL;
  
  if (commonNeighbours.size() == 2)
  {
    Edge* const pEdgeNaNb = pTriangleMesh->GetEdge(commonNeighbours.front(),
                                                   commonNeighbours.back());
    if (pEdgeNaNb != NULL &&
        pTriangleMesh->EdgeFlipNecessary(commonNeighbours.front(),
                                         commonNeighbours.back(),
                                         pClosestVertices[0],
                                         pClosestVertices[1]))
    {
      pTriangleMesh->DeleteEdge(pEdgeNaNb, false);
      pInterconnectingEdge = pTriangleMesh->NewEdge(pClosestVertices[0],
                                                    pClosestVertices[1]);
    }
    else if (pEdgeNaNb == NULL)
    {
      pInterconnectingEdge = pTriangleMesh->NewEdge(pClosestVertices[0],
                                                    pClosestVertices[1]);
    }
  }
  else
  {
    pInterconnectingEdge = pTriangleMesh->NewEdge(pClosestVertices[0],
                                                  pClosestVertices[1]);
  }
  
  return pInterconnectingEdge;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::CreateTriangles
(Edge* pInterconnectingEdge,
 const std::vector<Vertex*>& commonNeighbours,
 Vertex* pClosestVertices[2],
 TriangleMesh::SharedPtr pTriangleMesh,
 std::vector<Edge*>* pEdgesToRepair)
{
  const Vertex* const pVtxOfTri0 = (pInterconnectingEdge->GetT0() != NULL) ?
  pInterconnectingEdge->GetT0()->GetOther(pClosestVertices[0],
                                          pClosestVertices[1]) : NULL;
  const Vertex* const pVtxOfTri1 = (pInterconnectingEdge->GetT1() != NULL) ?
  pInterconnectingEdge->GetT1()->GetOther(pClosestVertices[0],
                                          pClosestVertices[1]) : NULL;
  BOOST_FOREACH(Vertex* pVN, commonNeighbours)
  {
    // create triangle only if it does not exist
    if ((pVN != pVtxOfTri0) && (pVN != pVtxOfTri1))
    {
      Edge* const pEdge0N = pTriangleMesh->GetEdge(pClosestVertices[0], pVN);
      Edge* const pEdge1N = pTriangleMesh->GetEdge(pClosestVertices[1], pVN);
      
      if ((pEdge0N->GetValence() == 2) && !(pEdge1N->GetValence() == 2))
      {
        Vertex* const pVL = pEdge0N->GetT0()->GetOther(pClosestVertices[0], pVN);
        Vertex* const pVR = pEdge0N->GetT1()->GetOther(pClosestVertices[0], pVN);
        const Flt existingRoughness =
        pTriangleMesh->GetRoughness(pClosestVertices[0], pVN,
                                    pVL, pVR);
        const Flt newRoughnessL1 =
        pTriangleMesh->GetRoughness(pClosestVertices[0], pVN,
                                    pVL, pClosestVertices[1]);
        const Flt newRoughnessR1 =
        pTriangleMesh->GetRoughness(pClosestVertices[0], pVN,
                                    pVR, pClosestVertices[1]);
        if (newRoughnessL1 < existingRoughness)
        {
          if (newRoughnessR1 < newRoughnessL1)
          {
            pTriangleMesh->DeleteTriangle(pEdge0N->GetT0(), pEdgesToRepair);
          }
          else
          {
            pTriangleMesh->DeleteTriangle(pEdge0N->GetT1(), pEdgesToRepair);
          }
        }
        else
        {
          if (newRoughnessR1 < existingRoughness)
          {
            pTriangleMesh->DeleteTriangle(pEdge0N->GetT0(), pEdgesToRepair);
          }
        }
      }
      else if (!(pEdge0N->GetValence() == 2) && (pEdge1N->GetValence() == 2))
      {
        Vertex* const pVL = pEdge1N->GetT0()->GetOther(pClosestVertices[1], pVN);
        Vertex* const pVR = pEdge1N->GetT1()->GetOther(pClosestVertices[1], pVN);
        const Flt existingRoughness =
        pTriangleMesh->GetRoughness(pClosestVertices[0], pVN,
                                    pVL, pVR);
        const Flt newRoughnessL0 =
        pTriangleMesh->GetRoughness(pClosestVertices[1], pVN,
                                    pVL, pClosestVertices[0]);
        const Flt newRoughnessR0 =
        pTriangleMesh->GetRoughness(pClosestVertices[1], pVN,
                                    pVR, pClosestVertices[0]);
        if (newRoughnessL0 < existingRoughness)
        {
          if (newRoughnessR0 < newRoughnessL0)
          {
            pTriangleMesh->DeleteTriangle(pEdge1N->GetT0(), pEdgesToRepair);
          }
          else
          {
            pTriangleMesh->DeleteTriangle(pEdge1N->GetT1(), pEdgesToRepair);
          }
        }
        else
        {
          if (newRoughnessR0 < existingRoughness)
          {
            pTriangleMesh->DeleteTriangle(pEdge1N->GetT0(), pEdgesToRepair);
          }
        }
      }
      else if ((pEdge0N->GetValence() == 2) && (pEdge1N->GetValence() == 2))
      {
        // possible configurations (0: pEdge0N, 1: pEdge1N)
        //   initial               variant1    variant2    variant3
        //  --0  1--              --0--1--      0--1--    --0--1
        //    |  |                              |              |
        
        typedef Flt VPC;
        
        Vertex* const pVtxC0 = pClosestVertices[0];
        Vertex* const pVtxC1 = pClosestVertices[1];
        
        Triangle* pTri00 = pEdge0N->GetT0();
        Triangle* pTri01 = pEdge0N->GetT1();
        Triangle* pTri10 = pEdge1N->GetT0();
        Triangle* pTri11 = pEdge1N->GetT1();
        
        const Vertex* const pVtx00 = pTri00->GetOther(pVtxC0, pVN);
        const Vertex* const pVtx01 = pTri01->GetOther(pVtxC0, pVN);
        const Vertex* const pVtx10 = pTri10->GetOther(pVtxC1, pVN);
        const Vertex* const pVtx11 = pTri11->GetOther(pVtxC1, pVN);
        
        const VPC rough00toE0to01 = pTriangleMesh->GetRoughness(pVtxC0, pVN,
                                                                pVtx00, pVtx01);
        const VPC rough00toE0toC1 = pTriangleMesh->GetRoughness(pVtxC0, pVN,
                                                                pVtx00, pVtxC1);
        const VPC rough01toE0toC1 = pTriangleMesh->GetRoughness(pVtxC0, pVN,
                                                                pVtx01, pVtxC1);
        const VPC rough11toE1to10 = pTriangleMesh->GetRoughness(pVtxC1, pVN,
                                                                pVtx11, pVtx10);
        const VPC roughC0toE1to10 = pTriangleMesh->GetRoughness(pVtxC1, pVN,
                                                                pVtxC0, pVtx10);
        const VPC roughC0toE1to11 = pTriangleMesh->GetRoughness(pVtxC1, pVN,
                                                                pVtxC0, pVtx11);
        
        const VPC roughInitial = rough00toE0to01 + rough11toE1to10;
        const VPC roughVariant1 = rough00toE0toC1 + roughC0toE1to10;
        const VPC roughVariant2 = rough01toE0toC1 + roughC0toE1to10;
        const VPC roughVariant3 = rough00toE0toC1 + roughC0toE1to11;
        
        if ((roughVariant1 < roughInitial) &&
            (roughVariant1 < roughVariant2) &&
            (roughVariant1 < roughVariant3))
        {
          pTriangleMesh->DeleteTriangle(pTri01, pEdgesToRepair);
          pTriangleMesh->DeleteTriangle(pTri11, pEdgesToRepair);
        }
        else if ((roughVariant2 < roughInitial) &&
                 (roughVariant2 < roughVariant1) &&
                 (roughVariant2 < roughVariant3))
        {
          pTriangleMesh->DeleteTriangle(pTri00, pEdgesToRepair);
          pTriangleMesh->DeleteTriangle(pTri11, pEdgesToRepair);
        }
        else if ((roughVariant3 < roughInitial) &&
                 (roughVariant3 < roughVariant2) &&
                 (roughVariant3 < roughVariant3))
        {
          pTriangleMesh->DeleteTriangle(pTri01, pEdgesToRepair);
          pTriangleMesh->DeleteTriangle(pTri10, pEdgesToRepair);
        }
      }
      pTriangleMesh->NewTriangle(pClosestVertices[0], pClosestVertices[1], pVN);
    }
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::QuadAtInterconnection
(Edge* pInterconnectingEdge,
 TriangleMesh::SharedPtr pTriangleMesh,
 std::vector<Edge*>* pEdgesToRepair)
{
  Edge* pConvexEdges[2] = { NULL, NULL };
  Vertex* pConvexVertices[3] = { NULL, NULL, NULL };
  bool foundConvexBorderTri =
  pTriangleMesh->GetConvexTriAtHole(pInterconnectingEdge,
                                    pConvexEdges, pConvexVertices);
  if (foundConvexBorderTri)
  {
    const Flt length2 =
      (pConvexVertices[0]->GetPosition() -
       pConvexVertices[2]->GetPosition()).GetSquaredLength();
    const Flt avgEdgeLengt2 =
      this->fpSurfaceReconstruction->GetTriangleMesh()->
        GetAverageEdgeLength();
    if (length2 < sc<Flt>(9.0) * avgEdgeLengt2)
    {
      Edge* pNewEdge = pTriangleMesh->NewEdge(pConvexVertices[0],
                                              pConvexVertices[2]);
      pTriangleMesh->NewTriangle(pConvexVertices[0],
                                 pConvexVertices[1],
                                 pConvexVertices[2]);
      pEdgesToRepair->push_back(pNewEdge);
    }
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::QuadLoop2
(FourLoop* pFourLoop,
 int index)
{
  const Vertex* const pCurrVtx = pFourLoop->pVertices[index];
  const Vertex* const pPrevVtx =
    pFourLoop->pVertices[(index == 0) ? 3 : (index - 1)];
  
  bool notYetTriangulated = true;
  if (index == 1 || index == 2)
  {
    BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pCurrVtx))
    {
      if (index == 1 && pAdjEdge->GetOther(pCurrVtx) == pFourLoop->pVertices[3])
      {
        notYetTriangulated = false;
        break;
      }
      if (index == 2 && pAdjEdge->GetOther(pCurrVtx) == pFourLoop->pVertices[0])
      {
        notYetTriangulated = false;
        break;
      }
    }
  }
  
  if (notYetTriangulated)
  {
    BOOST_FOREACH(Edge* pAdjEdge, Adjacent<Edge>::Around(pCurrVtx))
    {
      if (pAdjEdge->GetValence() < 2)
      {
        Vertex* const pNextVtx = pAdjEdge->GetOther(pCurrVtx);
        if (pNextVtx != pPrevVtx)
        {
          const Triangle* const pT0 = pAdjEdge->GetT0();
          const Triangle* const pT1 = pAdjEdge->GetT1();
          const Triangle* const pT = (pT0 != NULL) ? pT0 : pT1;
          bool carryOn = true;
          if (pT != NULL)
          {
            if (pT->GetOther(pCurrVtx, pNextVtx) == pPrevVtx)
            {
              carryOn = false;
            }
          }
          
          if (carryOn)
          {
            if (index < 2)
            {
              pFourLoop->pEdges[index] = pAdjEdge;
              pFourLoop->pVertices[index + 1] = pNextVtx;
              this->QuadLoop2(pFourLoop, index + 1);
            }
            else
            {
              if (pNextVtx == pFourLoop->pVertices[3])
              {
                pFourLoop->pEdges[index] = pAdjEdge;
                this->TriangulateQuadLoop(pFourLoop);
                break;
              }
            }
          }
        }
      }
    }
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::TriangulateQuadLoop
(FourLoop* pLoop)
{
  if (pLoop->pEdges[0]->GetValence() < 2 &&
      pLoop->pEdges[1]->GetValence() < 2 &&
      pLoop->pEdges[2]->GetValence() < 2 &&
      pLoop->pEdges[3]->GetValence() < 2)
  {
    const Flt lengthA2 =
      (pLoop->pVertices[0]->GetPosition() -
       pLoop->pVertices[2]->GetPosition()).GetSquaredLength();
    const Flt lengthB2 =
      (pLoop->pVertices[1]->GetPosition() -
       pLoop->pVertices[3]->GetPosition()).GetSquaredLength();
    const Flt avgEdgeLengt2 =
      this->fpSurfaceReconstruction->GetTriangleMesh()->GetAverageEdgeLength();
    
    if (lengthA2 < sc<Flt>(9.0) * avgEdgeLengt2 &&
        lengthB2 < sc<Flt>(9.0) * avgEdgeLengt2)
    {
      Flt roughness02 =
        this->fpSurfaceReconstruction->GetTriangleMesh()->GetRoughness(
          pLoop->pVertices[0], pLoop->pVertices[2],
          pLoop->pVertices[1], pLoop->pVertices[3]);
      Flt roughness13 =
        this->fpSurfaceReconstruction->GetTriangleMesh()->GetRoughness(
          pLoop->pVertices[1], pLoop->pVertices[3],
          pLoop->pVertices[0], pLoop->pVertices[2]);
      
      if (roughness02 < roughness13)
      {
        Edge* const pNewEdge =
          this->fpSurfaceReconstruction->GetTriangleMesh()->NewEdge(
            pLoop->pVertices[0], pLoop->pVertices[2]);
        pNewEdge->SetAge(static_cast<Edge::Age>(0));
        this->fpSurfaceReconstruction->GetTriangleMesh()->NewTriangle(
          pLoop->pVertices[0], pLoop->pVertices[1], pLoop->pVertices[2]);
        this->fpSurfaceReconstruction->GetTriangleMesh()->NewTriangle(
          pLoop->pVertices[0], pLoop->pVertices[2], pLoop->pVertices[3]);
      }
      else
      {
        Edge* const pNewEdge =
          this->fpSurfaceReconstruction->GetTriangleMesh()->NewEdge(
            pLoop->pVertices[1], pLoop->pVertices[3]);
        pNewEdge->SetAge(static_cast<Edge::Age>(0));
        this->fpSurfaceReconstruction->GetTriangleMesh()->NewTriangle(
          pLoop->pVertices[0], pLoop->pVertices[1], pLoop->pVertices[3]);
        this->fpSurfaceReconstruction->GetTriangleMesh()->NewTriangle(
          pLoop->pVertices[1], pLoop->pVertices[2], pLoop->pVertices[3]);
      }
    }
  }
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
void
ReconstructionWorker::PrintStatistics
()
{
  SurfaceReconstruction* const pSurfRec = this->fpSurfaceReconstruction;
  std::cout << "POINTS: ";
  std::cout << pSurfRec->GetPointCloud()->GetNumOfPoints();
  std::cout << " ";
  std::cout << "iterations: ";
  std::cout << this->fpSurfaceReconstruction->GetIterationsDone() << " ";
  std::cout << "time: ";
  std::cout << this->fStopWatch.ElapsedMilliseconds() << " ";
  std::cout << "vertices: ";
  std::cout << pSurfRec->GetTriangleMesh()->GetNumOfVertices();
  std::cout << " ";
  std::cout << "faces: ";
  std::cout << pSurfRec->GetTriangleMesh()->GetNumOfTriangles();
  std::cout << std::endl;
}
