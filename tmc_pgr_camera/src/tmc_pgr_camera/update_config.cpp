/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <filesystem>
#include <fstream>
#include <iostream>

#include <string>

#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: $ rosrun tmc_pgr_camera update_config /home/hoge/target.yml\n"
             "             Or $ sudo ./update_config /etc/hoge/target.yml" << std::endl;
    return EXIT_FAILURE;
  }

  // Backup
  try {
    const std::filesystem::path src(argv[1]);
    const std::filesystem::path dst(std::string(argv[1]) + std::string("_bak"));
    std::filesystem::copy_file(src, dst);
  } catch (const std::filesystem::filesystem_error& e) {
    std::cerr << "Backup error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Backup." << std::endl;

  // Update
  try {
    YAML::Node node;
    node = YAML::LoadFile(argv[1]);
    const YAML::Node& cameras = node["cameras"];
    YAML::Emitter new_node;
    new_node << YAML::Comment(" Copyright (C) TOYOTA Motor Corporation");
    new_node << YAML::BeginMap;
    new_node << YAML::Key << "cameras";
    new_node << cameras;
    new_node << YAML::EndMap;

    std::ofstream output(argv[1]);
    output << new_node.c_str();
  } catch (const YAML::Exception& e) {
    std::cerr << "Failed to read file. File: " << argv[1] << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Failed to process: " << argv[1] << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Update." << std::endl;
  return EXIT_SUCCESS;
}
