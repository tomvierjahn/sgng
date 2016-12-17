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
#include <iostream>
#include <string>

#include "program_options.h"

boost::program_options::variables_map ProgramOptions::sVariablesMap;





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void 
ProgramOptions::Parse
(int argc, char* ppArgv[])
{
  namespace po = boost::program_options;
  
  // Define options
  po::options_description genericOptions("Generic Options");
  genericOptions.add_options()
    ("help,h",
     "produce help message")
    ("config,f",
     po::value<std::string>()->default_value("surface-reconstruction.cfg"),
     "name (and path) of configuration file");
  
  
  po::options_description configOptions("Configuration");
  configOptions.add_options()
    ("user-config,u",
     po::value<std::string>()->default_value("surface-reconstruction-user.cfg"),
     "name (and path) of user configuration file")
    ("project-config,p",
     po::value<std::string>()->default_value(""),
     "name (and path) of project configuration file")
    ("ttw",
     po::value<unsigned int>()->default_value(409),
     "Width of each texture tile inside large texture (default: 409)")
    ("tth",
     po::value<unsigned int>()->default_value(409),
     "Height of each texture tile inside large texture (default: 409)")
    ("tw",
     po::value<unsigned int>()->default_value(4096),
     "Width of large texture (default: 4096)")
    ("th",
     po::value<unsigned int>()->default_value(409),
     "Height of large texture (default: 4096)")
    ("ppv",
     po::value<unsigned int>()->default_value(100),
     "The number of points per vertex for activity updates (default: 100)")
    ("pvr",
     po::value<float>()->default_value(-1.0),
     "The ratio of points to vertices.")
    ("nv",
     po::value<unsigned int>()->default_value(0),
     "The number of vertices to reconstruct (default: 0)")
    ("tpp",
     po::value<unsigned int>()->default_value(100),
     "The number of texture coordinates per point/vertex (default: 100)")
    ("gla",
     po::value<std::string>()->default_value(""),
     "String of parameters for gluLookAt(...) to set up a fixed camera.")
    ("ttmat",
     po::value<std::string>()->default_value(""),
     "String of parameters for turntable animation (4x4 matrix + radius).")
    ("repetition",
     po::value<std::string>()->default_value("0"),
     "Specifies the repetition for mesh export")
    ("filePrefix",
     po::value<std::string>()->default_value(""),
     "The path to search for input directories")
    ("exportPrefix",
     po::value<std::string>()->default_value(""),
     "The path to export reconstructions to.")
    ("lastFrame",
     po::value<int>()->default_value(0),
     "The last frame that was exported")
    ("lastVertices",
     po::value<int>()->default_value(0),
     "The last frame that was exported")
    ("beta",
     po::value<float>()->default_value(0.1f),
     "Step size best match")
    ("eta",
     po::value<float>()->default_value(0.01f),
     "Step size second best match")
    ("lambda",
     po::value<unsigned int>()->default_value(100),
     "Vertex addition/removal every \\lambda-th iteration")
    ("vtxInactivityThreshold",
     po::value<float>()->default_value(12.0f),
     "Vertex inactivity threshold (multiple of |V|)")
    ("amax",
     po::value<unsigned int>()->default_value(20),
     "Maximum edge penalty")
    ("maxLen",
     po::value<float>()->default_value(3.0),
     "Length threshold for edges")
    ("killTime",
     po::value<unsigned int>()->default_value(0),
     "Kill reconstruction after seconds")
#ifndef NO_FRAMELIST
    ("frameListFile",
     po::value<std::string>()->default_value(""),
     "file listing frames for export during reconstruction")
#endif
#ifndef NO_TEXTURES
    ("texwidth",
     po::value<unsigned int>()->default_value(1),
     "width of texture images")
    ("texheight",
     po::value<unsigned int>()->default_value(1),
     "height of texture images")
#endif
  ;
  
  
  po::options_description fileOptions("File Options");
  fileOptions.add_options()
    ("obb",
     po::value<std::string>()->default_value(""),
     "file containing bounding boxes to suppress outliers")
    ("mesh,m",
     po::value<std::string>()->default_value(""),
     "file to store a mesh")
    ("remotePrefix",
     po::value<std::string>()->default_value(""),
     "prefix of remote storage")
    ("localPrefix",
     po::value<std::string>()->default_value(""),
     "prefix of local storage");

  po::options_description otherOptions("Other");
  otherOptions.add_options()
    ("verbosity,v",
     po::value<int>()->default_value(3),
     "Verbosity level:\n"
     "    0 - quiet \n"
     "    1 - print error only \n"
     "    2 - as level 1 + warnings \n" 
     "    3 - as level 1 + infos \n" 
     "    4 - as level 2 + debug messages")
    ("seed,S",
     po::value<unsigned int>()->default_value(0),
     "Random seed");
  
  po::options_description hiddenOptions("Hidden options");
  hiddenOptions.add_options()
    ("input-file", po::value<std::string>(), "input file");
  po::positional_options_description positionalOptions;
  positionalOptions.add("input-file", 1);
  
  po::options_description cmdLineOptions("triangulation command line options");
  cmdLineOptions
    .add(genericOptions)
    .add(fileOptions)
    .add(otherOptions)
    .add(configOptions)
  .add(hiddenOptions);
  po::options_description cmdLineOptionsVisible(
    "triangulation command line options");
  cmdLineOptionsVisible
    .add(genericOptions)
    .add(fileOptions)
    .add(otherOptions)
  .add(configOptions)
  .add(hiddenOptions);
  
  po::options_description configFileOptions;
  configFileOptions
    .add(fileOptions)
    .add(otherOptions)
    .add(configOptions)
    .add(hiddenOptions);
  
  
  // Parse command line options
  po::store(po::command_line_parser(argc, ppArgv)
            .options(cmdLineOptions)
            .positional(positionalOptions)
            .run(),
            ProgramOptions::sVariablesMap);
  po::notify(ProgramOptions::sVariablesMap);
  
  if (HaveOption("help"))
  {
    std::cout << cmdLineOptionsVisible << std::endl;
    exit(0);
  }
  
  // Parse config file options
  std::string configFile = ProgramOptions::GetOption<std::string>("config");
  std::ifstream configFileStream(configFile.c_str());
  if (!configFileStream)
  {
    std::cout << "Cannot open config file: '" 
              << configFile << "'." << std::endl;
  }
  else
  {
    std::cout << "Using config file: '" << configFile << "'." << std::endl;
    po::store(
      po::parse_config_file(configFileStream, configFileOptions),
      ProgramOptions::sVariablesMap);
    po::notify(ProgramOptions::sVariablesMap);
  }
  
  
  // Parse user config file options
  std::string userConfigFile = ProgramOptions::GetOption<std::string>("user-config");
  std::ifstream userConfigFileStream(userConfigFile.c_str());
  if (userConfigFileStream)
  {
    std::cout << "Using user config file '"
              << userConfigFile << "'."
              << std::endl;
    
    try{
      po::parsed_options parsedOptions = 
      po::parse_config_file(userConfigFileStream, configFileOptions, true);
      po::store(parsedOptions, ProgramOptions::sVariablesMap);
      po::notify(ProgramOptions::sVariablesMap);
      
      // Collect unrecognised options
      std::vector<std::string> unrecognisedOptions = 
        po::collect_unrecognized(parsedOptions.options, po::include_positional);
      
      ProgramOptions::PrintUnrecognisedOptions(unrecognisedOptions);  
    }
    catch(...)
    {
      std::cout << "[ERROR] Could not parse user config file" << std::endl;
    }
  }
  
  // parse project config
  if (ProgramOptions::HaveOption("project-config"))
  {
    // Parse project config file options
    std::string projectConfigFile =
      ProgramOptions::GetOption<std::string>("project-config");
    std::ifstream projectConfigFileStream(projectConfigFile.c_str());
    if (projectConfigFileStream)
    {
      std::cout << "Using project config file '"
        << projectConfigFile << "'."
        << std::endl;
      
      try{
        po::parsed_options parsedOptions =
        po::parse_config_file(projectConfigFileStream, configFileOptions, true);
        po::store(parsedOptions, ProgramOptions::sVariablesMap);
        po::notify(ProgramOptions::sVariablesMap);
        
        // Collect unrecognised options
        std::vector<std::string> unrecognisedOptions =
        po::collect_unrecognized(parsedOptions.options, po::include_positional);
        
        ProgramOptions::PrintUnrecognisedOptions(unrecognisedOptions);
      }
      catch(...)
      {
        std::cout << "[ERROR] Could not parse project config file" << std::endl;
      }
    }
  }
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
void 
ProgramOptions::PrintUnrecognisedOptions
(const std::vector<std::string>& unrecognisedOptions)
{
  // unrecognisedOptions contains a list of the format:
  // "program option, value, program option, value, ..."
  // We just want to warn about the program options,
  // so skip every second element.
  std::vector<std::string>::const_iterator itor;  
  for (itor = unrecognisedOptions.begin(); 
       itor != unrecognisedOptions.end(); 
       itor += 2)
  {
    std::cout << "[WARNING]: Did not recognise program option '" 
              << (*itor) << "'" << std::endl;
  }
}
