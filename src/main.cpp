//
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c), assimp OpenGL sample
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

// ----------------------------------------------------------------------------
// Simple sample to prove that Assimp is easy to use with OpenGL.
// It takes a file name as command line parameter, loads it using standard
// settings and displays it.
// ----------------------------------------------------------------------------

#define GL_GLEXT_PROTOTYPES

#include <iostream>
#include <stdlib.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <opencv2/highgui/highgui.hpp>

#if USE_RENDERER_GLUT
#include <object_recognition_renderer/renderer_glut.h>
#else
#include <object_recognition_renderer/renderer_osmesa.h>
#endif

int
main(int argc, char **argv)
{
  // Parse arguments
  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("width", po::value<int>()->default_value(640), "image width")
    ("height", po::value<int>()->default_value(480), "image height")
    ("near", po::value<double>()->default_value(0.1), "renderer near plane")
    ("far", po::value<double>()->default_value(1000.0), "renderer far plane")
    ("fx", po::value<double>()->default_value(525), "focal length (x)")
    ("fy", po::value<double>()->default_value(525), "focal length (y)")
    ("prefix", po::value<std::string>()->default_value(std::string("")), "focal length (y)")
    ("verbose", po::value<int>()->default_value(1), "verbose output (statusbar, etc)")
    ("mesh-file", po::value<std::string>()->required(), "mesh to use to generate views")
    ("steps", po::value<int>()->default_value(150), "number of render steps")
    ;

  po::positional_options_description pdesc;
  pdesc.add("mesh-file", 1);

  po::variables_map vm;
  //po::store(po::parse_command_line(argc, argv, desc), vm);
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pdesc).run(), vm);
  try {

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 1;
    }

    po::notify(vm);    
  } catch(boost::program_options::error &ex) {
    std::cerr << "Invalid arguments: " << ex.what() << std::endl;
    return -1;
  }

  // Define the display
  int width = vm["width"].as<int>(),
      height = vm["height"].as<int>();
  double near = vm["near"].as<double>(),
         far = vm["far"].as<double>();
  double focal_length_x = vm["fx"].as<double>(),
         focal_length_y = vm["fy"].as<double>();
  bool verbose = (bool)vm["verbose"].as<int>();
  int steps = vm["steps"].as<int>();

  // the model name can be specified on the command line.
#if USE_RENDERER_GLUT
  RendererGlut renderer = RendererGlut(vm["mesh-file"].as<std::string>());
#else
  RendererOSMesa renderer = RendererOSMesa(vm["mesh-file"].as<std::string>());
#endif

  renderer.set_parameters(width, height, focal_length_x, focal_length_y, near, far);

  RendererIterator renderer_iterator = RendererIterator(&renderer, steps);

  cv::Mat image, depth, mask;
  cv::Matx33d R;
  cv::Vec3d T;

  // Open the yaml file
  cv::FileStorage poses_f("render_info.yml", cv::FileStorage::WRITE);

  // Store the configuration
  poses_f 
    << "width" << width
    << "height" << height
    << "near" << near
    << "far" << far
    << "fx" << focal_length_x
    << "fy" << focal_length_y;

  // Store the final number of poses in the yaml file
  int n_templates = (int)renderer_iterator.n_templates();
  poses_f << "n_poses" << n_templates;
  poses_f << "filename_format" << "%05d";

  // Begin the list of poses in the yaml file
  poses_f << "poses" << "[";

  size_t i = 0;

  for (i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator)
  {
    // Output status
    if(verbose) {
      std::cout<<"\x1B[2K"<<"\x1B[0E";
      std::cout<<(boost::format("%6.2f%% complete...") % (100.0*i/double(n_templates)));
      std::flush(std::cout);
    }

    renderer_iterator.render(image, depth, mask);

    // Get transform
    R = renderer_iterator.R();
    T = renderer_iterator.T();

    // Store the transform in the yaml file
    poses_f << "{:";

    poses_f << "R" << "[:";
    for(int r=0; r<3; r++) {
      for(int c=0; c<3; c++) {
        poses_f << R(r,c);
      }
    }
    poses_f << "]";

    poses_f << "T" << "[:";
    for(int c=0; c<3; c++) {
      poses_f << T(c);
    }
    poses_f <<"]";

    poses_f << "}";

    // Write out the images
    cv::imwrite(boost::str(boost::format("depth_%05d.png") % (i)), depth);
    cv::imwrite(boost::str(boost::format("image_%05d.png") % (i)), image);
    cv::imwrite(boost::str(boost::format("mask_%05d.png") % (i)), mask);
  }

  poses_f << "]";

  poses_f.release();

  std::cout<<std::endl;

  return 0;
}
