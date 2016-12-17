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

#ifndef SURFACE_RECONSTRUCTION__SURFACE_RECONSTRUCTION_H_
#define SURFACE_RECONSTRUCTION__SURFACE_RECONSTRUCTION_H_

#include <string>
#include <list>

#ifndef Q_MOC_RUN
#include <boost/filesystem/path.hpp>
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <no_warning/boost__range__iterator_range.h>
#include <no_warning/boost__thread__mutex.h>
#endif

#include <io/input_data.h>

#include "data_types.h"
#include "point_cloud.h"
#include "reconstruction_worker.h"
#include "textures.h"
#include "triangle_mesh.h"

#include "io/data_storage.h"




class SurfaceReconstruction;


////////////////////////////////////////////////////////////////////////////////
/// The application's main class.
/// Holds all data.
////////////////////////////////////////////////////////////////////////////////
class SurfaceReconstruction
{

public:
  // for convenience
  typedef boost::shared_ptr<SurfaceReconstruction> SharedPtr;
  typedef boost::timed_mutex Mutex;

  SurfaceReconstruction();
  
  void Init();
  void InitSGNG();

  void LoadInputFile(const std::string& inputFile);
  void ReloadInputFile();
  void ReloadDenseInputFile();

  void Statistics();
  
  void SaveTriangleMesh();
  
  void ExportObj(const std::string& filename);
#ifndef NO_TEXTURES
  void ExportTexturedObj(const std::string& filename);
#endif
#ifndef IGNORE_COLOR
  void ExportColoredPly(const std::string& filename);
#endif
  
  void Reconstruct(unsigned int numOfIterations,
                   boost::function<void()> finishedCallback = 
                     boost::function<void()>(),
                   boost::function<void(const std::string&)> exportCallback =
                     boost::function<void(const std::string&)>());
  void ReconstructStandalone();
  void Stop();
  
  PointCloud::SharedPtr GetPointCloud() { return this->fpPointCloud; }
  PointCloud::SharedPtr GetDensePointCloud() { return this->fpDensePointCloud; }
  TriangleMesh::SharedPtr GetTriangleMesh() { return this->fpTriangleMesh; }
  Textures::SharedPtr GetTextures() { return this->fpTextures; }
  
  void LockMesh() { this->fTriangleMeshMutex.lock(); }
  bool TimedLockMesh(unsigned int time)
  {
	  using boost::posix_time::milliseconds;
	  return this->fTriangleMeshMutex.timed_lock(milliseconds(time));
  }
  void UnlockMesh() { this->fTriangleMeshMutex.unlock(); }

  void LockPoints() { this->fPointCloudMutex.lock(); }
  bool TimedLockPoints(unsigned int time)
  {
	  using boost::posix_time::milliseconds;
	  return this->fPointCloudMutex.timed_lock(milliseconds(time));
  }
  void UnlockPoints() { this->fPointCloudMutex.unlock(); }

  void LockTextures() { this->fTexturesMutex.lock(); }
  bool TimedLockTextures(unsigned int time)
  {
	  using boost::posix_time::milliseconds;
	  return this->fTexturesMutex.timed_lock(milliseconds(time));
  }
  void UnlockTextures() { this->fTexturesMutex.unlock(); }
  
  
  void LockMutexForExportWhileLearning() { this->fMutexForExportWhileLearning.lock(); }
  void UnlockMutexForExportWhileLearning() { this->fMutexForExportWhileLearning.unlock(); }
  
  
  void IncrementIterationsDone() { ++this->fIterationsDone; }
  unsigned long GetIterationsDone() const { return this->fIterationsDone; }
  
  void SetLastSignal(const Point* pS) { this->fpLastSignal = pS; }
  
  void SetLastClosestVertices0(const Vertex* pV)
  {
    this->fpLastClosestVertices0 = pV;
  }
  
  void SetLastClosestVertices1(const Vertex* pV)
  {
    this->fpLastClosestVertices1 = pV;
  }

  const Point* GetLastSignal() const { return this->fpLastSignal; }
  const Vertex* GetLastClosestVertices1() const
  {
    return this->fpLastClosestVertices0;
  }
  const Vertex* GetLastClosestVertices0() const
  {
    return this->fpLastClosestVertices1;
  }
  
  std::string GetPointCloudStorageInfo() const
  {
    return this->fInputData.GetInfo();
  }
  
  std::string GetMeshStorageInfo() const
  {
    if (this->fpMeshStorage.get() != NULL)
    {
      return this->fpMeshStorage->GetInfo();
    }
    else
    {
      return std::string("");
    }
  }

  std::string CreateTimestampStr();
  
  void Reset();
  
  void InitForRenderedReconstruction();
  void DoRenderedReconstruction();
  
  unsigned int PeekNextExportByIteration() const
  {
    unsigned int retVal = 0u - 1;
    this->fNextExportByIterationMutex.lock();
    if (this->fNextExportByIteration.size() > 0)
    {
      retVal = this->fNextExportByIteration.front();
    }
    this->fNextExportByIterationMutex.unlock();
    return retVal;
  }

  unsigned int TakeNextExportByIteration()
  {
    unsigned int retVal = 0u - 1;
    this->fNextExportByIterationMutex.lock();
    if (this->fNextExportByIteration.size() > 0)
    {
      retVal = this->fNextExportByIteration.front();
      this->fNextExportByIteration.pop_front();
    }
    this->fNextExportByIterationMutex.unlock();
    return retVal;
  }
  
  bool NextExportByIterationAvailable() const
  {
    bool retVal = false;
    this->fNextExportByIterationMutex.lock();
    retVal = this->fNextExportByIteration.size() > 0;
    this->fNextExportByIterationMutex.unlock();
    return retVal;
  }
  
  const boost::filesystem::path& GetLocalPrefix() const
  {
    return this->fLocalPrefix;
  }
  
  const boost::filesystem::path& GetPointPath() const
  {
    return this->fPointPath;
  }
  
  unsigned int GetTargetNumVertices() const { return this->fTargetNumVertices; }
  
private:
  PointCloud::SharedPtr   fpPointCloud;     ///< \brief Shared pointer to input point cloud
  PointCloud::SharedPtr   fpDensePointCloud;
  TriangleMesh::SharedPtr fpTriangleMesh;   ///< \brief Shared pointer to reconstructed triangle mesh
  Textures::SharedPtr fpTextures;           ///< \brief Shared pointer to textures
  
  Mutex fTriangleMeshMutex;   ///< \brief Mutex to synchronize rendering thread
  Mutex fPointCloudMutex;     ///< \brief Mutex to synchronize rendering thread
  Mutex fTexturesMutex;       ///< \brief Mutex to synchronize rendering thread
  
  boost::filesystem::path fRemotePrefix;
  boost::filesystem::path fLocalPrefix;
  boost::filesystem::path fPointPath;
  
  io::InputData fInputData;   ///< \brief Handles input data loading
  io::InputData fDenseInputData;
  
  io::DataStorage<TriangleMesh>::SharedPtr fpMeshStorage;       ///< \brief SharedPtr to storage for output mesh
  
  ReconstructionWorker::SharedPtr fpReconstructionWorker;   ///< \brief SharedPtr to reconstruction worker thread
  
  unsigned long fIterationsDone;  ///< \brief Counter to keep track of the number of samples presented, yet
  
  const Point*  fpLastSignal;           ///< \brief Info to highlight signal in visualization
  const Vertex* fpLastClosestVertices0; ///< \brief Info to highlight best matching unit in visualization
  const Vertex* fpLastClosestVertices1; ///< \brief Info to highlight second best matching unit in visualization
  
  bool fInitializedSGNG;
  
  std::list<unsigned int> fNextExportByIteration;
  mutable boost::mutex fNextExportByIterationMutex;
  mutable boost::mutex fMutexForExportWhileLearning;
  
  unsigned int fTargetNumVertices;
};  // class SurfaceReconstruction


#endif  // #ifndef SURFACE_RECONSTRUCTION__SURFACE_RECONSTRUCTION_H_
