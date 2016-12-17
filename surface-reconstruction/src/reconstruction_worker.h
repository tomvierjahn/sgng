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

#ifndef SURFACE_RECONSTRUCTION__RECONSTRUCTION_WORKER_H_
#define SURFACE_RECONSTRUCTION__RECONSTRUCTION_WORKER_H_


#include <list>
#include <string>
#include <utility>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#endif

#include <common/stop_watch.h>

#include <no_warning/boost__thread.h>

#include "triangle_mesh.h"


class Edge;
class SurfaceReconstruction;
class Vertex;



////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
class ReconstructionWorker
{
public:
  // convenience
  typedef boost::shared_ptr<ReconstructionWorker> SharedPtr;
  
  ReconstructionWorker(SurfaceReconstruction* pSurfaceReconstruction);
  
  void Stop() { this->fRunning = false; }
  
  void Run(boost::function<void()> finishedCallback =
             boost::function<void()>(),
           boost::function<void(const std::string&)> exportCallback =
             boost::function<void(const std::string&)>());
  
  Flt GetBeta() const { return this->fBeta; }
  Flt GetEta() const { return this->fEta; }
  unsigned int GetLambda() const { return this->fLambda; }
  Flt GetVertexInactivityThreshold() const { return this->fVertexInactivityThreshold; }
  unsigned int GetMaxEdgePenalty() const { return this->fMaxEdgePenalty; }
  Flt GetMaxLen2() const { return this->fMaxLen2; }
  float GetTime() { return this->fStopWatch.ElapsedMilliseconds() * 0.001f; }
  
private:
  void DoSingleReconstructionStep();
  void UpdateVertex(Vertex* pVertex,
                    Flt learningRate,
                    const Point* pSamplePoint,
                    TriangleMesh* pTriMesh,
                    bool isNeighbour = false);
  void UpdateNeighbours(Vertex* pAroundVertex,
                        Flt learningRate,
                        const Point* pSamplePoint,
                        TriangleMesh* pTriMesh);
  
  float MaxEdgeDistance(Triangle* pTriangle, const Point* pSamplePoint);
  void FitBarycentric(Triangle* pTriangle, Flt rate, const Point* pSamplePoint, TriangleMesh* pTriMesh);
  
  bool ThalesCircleCheck(const Edge* pEdge, const Vertex* pAroundVertex);
  
  void DoEdgeSplit();
  void DoVertexRemoval();
  
  Edge* FindInterconnectinEdge(Vertex* pVtxA, Vertex* pVtxB);
  Edge* ProcessEdgeStar(Vertex* pClosestVertices[2],
                        const Point* pSamplePoint,
                        TriangleMesh* pTriangleMesh,
                        std::vector<Edge*>* pEdgesToRepair);
  void UpdateVertexActivity(Vertex* pVtx,
                            TriangleMesh* pTriangleMesh);
  
  Edge* FindEdgeToCollapse(Vertex* pVtxSrc,
                           TriangleMesh* pTriangleMesh);
  void DoEdgeAgeingAndDeletion(Vertex* pBmu,
                               Edge* pInterconnectingEdge,
                               TriangleMesh* pTriangleMesh,
                               std::vector<Edge*>* pEdgesToRepair);
  
  Edge* CreateOrFlipInterconnection(
    const std::vector<Vertex*>& commonNeighbours,
    Vertex* pClosestVertices[2],
    TriangleMesh::SharedPtr pTriangleMesh);
  
  void CreateTriangles(Edge* pInterconnectingEdge,
                       const std::vector<Vertex*>& commonNeighbours,
                       Vertex* pClosestVertices[2],
                       TriangleMesh::SharedPtr pTriangleMesh,
                       std::vector<Edge*>* pEdgesToRepair);
  
  void QuadAtInterconnection(Edge* pInterconnectingEdge,
                             TriangleMesh::SharedPtr pTriangleMesh,
                             std::vector<Edge*>* pEdgesToRepair);
  
  struct FourLoop
  {
    FourLoop(Vertex* pV0, Vertex* pV1, Vertex* pV2, Vertex* pV3,
             Edge* pE0, Edge* pE1, Edge* pE2, Edge* pE3)
    {
      pVertices[0] = pV0;
      pVertices[1] = pV1;
      pVertices[2] = pV2;
      pVertices[3] = pV3;
      pEdges[0] = pE0;
      pEdges[1] = pE1;
      pEdges[2] = pE2;
      pEdges[3] = pE3;
    }
    Vertex* pVertices[4];
    Edge* pEdges[4];
  };
  void QuadLoop2(FourLoop* pFourLoop,
                 int index);
  void TriangulateQuadLoop(FourLoop* pLoop);
  
  void PrintStatistics();
  
  SurfaceReconstruction* fpSurfaceReconstruction;

  bool fRunning;
  boost::random::mt11213b fRng;
  StopWatch fStopWatch;
  
  Flt fBeta;
  Flt fEta;
  unsigned int fLambda;
  Flt fVertexInactivityThreshold;
  unsigned int fMaxEdgePenalty;
  Flt fMaxLen2;
  int fDenseIterations;
  
#ifndef NO_FRAMELIST
  std::list<std::pair<unsigned int, unsigned int> > fFrameIterations;
#endif
};  // class ReconstructionWorker


#endif  // #ifndef SURFACE_RECONSTRUCTION__RECONSTRUCTION_WORKER_H_
