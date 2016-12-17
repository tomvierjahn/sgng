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

#ifndef SURFACE_RECONSTRUCTION__PROGRAM_OPTIONS_H_
#define SURFACE_RECONSTRUCTION__PROGRAM_OPTIONS_H_

#include <string>

#ifndef Q_MOC_RUN
#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#endif

#include <common/logger.h>


class Logger;


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
class ProgramOptions
{
public:
  static void Parse(int argc, char* ppArgv[]);
  
  static Logger::LogLevel GetMaximumLogLevel()
  {
    return static_cast<Logger::LogLevel>(
      ProgramOptions::GetOption<int>("verbosity"));
  }
  
  static std::string GetInputFile()
  {
    return (ProgramOptions::HaveOption("input-file") ?
            ProgramOptions::GetOption<std::string>("input-file") :
            std::string(""));
  }
  
  static std::string GetOutlierBoundingFile()
  {
    return (ProgramOptions::HaveOption("obb") ?
            ProgramOptions::GetOption<std::string>("obb") :
            std::string(""));
  }
  
#ifndef NO_FRAMELIST
  static std::string GetFrameListFileFile()
  {
    return (ProgramOptions::HaveOption("frameListFile") ?
            ProgramOptions::GetOption<std::string>("frameListFile") :
            std::string(""));
  }
#endif

#ifndef NO_TEXTURES
  static unsigned int GetTextureWidth()
  {
    return ProgramOptions::GetOption<unsigned int>("texwidth");
  }
  static unsigned int GetTextureHeight()
  {
    return ProgramOptions::GetOption<unsigned int>("texheight");
  }
#endif
  
  static std::string GetMeshFile()
  {
    return ProgramOptions::GetOption<std::string>("mesh");
  }
  
  static std::string GetRemotePrefix()
  {
    return ProgramOptions::GetOption<std::string>("remotePrefix");
  }
  
  static std::string GetLocalPrefix()
  {
    return ProgramOptions::GetOption<std::string>("localPrefix");
  }
  
  static std::string GetGla()
  {
    return ProgramOptions::GetOption<std::string>("gla");
  }
  
  static std::string GetTTMat()
  {
    return ProgramOptions::GetOption<std::string>("ttmat");
  }
  
  static unsigned int GetRandomSeed()
  {
    return ProgramOptions::GetOption<unsigned int>("seed");
  }

  static unsigned int GetTextureTileWidth()
  {
    return ProgramOptions::GetOption<unsigned int>("ttw");
  }

  static unsigned int GetTextureTileHeight()
  {
    return ProgramOptions::GetOption<unsigned int>("tth");
  }
  
  static unsigned int GetTotalTextureWidth()
  {
    return ProgramOptions::GetOption<unsigned int>("tw");
  }
  
  static unsigned int GetTotalTextureHeight()
  {
    return ProgramOptions::GetOption<unsigned int>("th");
  }
  
  static unsigned int GetPointsPerVertex()
  {
    return ProgramOptions::GetOption<unsigned int>("ppv");
  }
  
  static float PointVertexRatio()
  {
    return ProgramOptions::GetOption<float>("pvr");
  }
  
  static unsigned int NumVertices()
  {
    return ProgramOptions::GetOption<unsigned int>("nv");
  }
  
  static unsigned int GetTexturesPerPoint()
  {
    return ProgramOptions::GetOption<unsigned int>("tpp");
  }
  
  static std::string GetNotifyHost()
  {
    return ProgramOptions::GetOption<std::string>("notifyHost");
  }
  
  static int GetNotifyPort()
  {
    return ProgramOptions::GetOption<int>("notifyPort");
  }
  
  static std::string GetRepetition()
  {
    return ProgramOptions::GetOption<std::string>("repetition");
  }
  
  static std::string GetFilePrefix()
  {
    return ProgramOptions::GetOption<std::string>("filePrefix");
  }
  
  static std::string GetExportPrefix()
  {
    return ProgramOptions::GetOption<std::string>("exportPrefix");
  }
  
  static int GetLastFrame()
  {
    return ProgramOptions::GetOption<int>("lastFrame");
  }
  
  static int GetLastVertices()
  {
    return ProgramOptions::GetOption<int>("lastVertices");
  }
  
  static float GetBeta()
  {
    return ProgramOptions::GetOption<float>("beta");
  }
  
  static float GetEta()
  {
    return ProgramOptions::GetOption<float>("eta");
  }
  
  static float GetVtxInactivityThreshold()
  {
    return ProgramOptions::GetOption<float>("vtxInactivityThreshold");
  }
  
  static unsigned int GetLambda()
  {
    return ProgramOptions::GetOption<unsigned int>("lambda");
  }
  
  static unsigned int GetAmax()
  {
    return ProgramOptions::GetOption<unsigned int>("amax");
  }
  
  static float GetMaxLen()
  {
    return ProgramOptions::GetOption<float>("maxLen");
  }
  
  static unsigned int GetKillTime()
  {
    return ProgramOptions::GetOption<unsigned int>("killTime");
  }


  

private:
  ProgramOptions() {};
  
  static void PrintUnrecognisedOptions(
    const std::vector<std::string>& unrecognisedOptions);
  
  static bool HaveOption(const std::string& name)
  {
    return ProgramOptions::sVariablesMap.count(name) > 0;
  }
  
  template<class T> static T GetOption(const std::string& name)
  {
    return ProgramOptions::sVariablesMap[name].as<T>();
  }
  
  static boost::program_options::variables_map sVariablesMap;
  static const char* const spVersionString;
};

#endif // #ifndef SURFACE_RECONSTRUCTION__PROGRAM_OPTIONS_H_
