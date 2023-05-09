#include <chrono>
#include <filesystem>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include "motor/RMDS.h"

int main(int argc, char** argv) {
  std::string dev("");
  if (argv[1][0] == '/') {
    dev = std::string(argv[1]);
  } else {
    dev = Motor::RMDS::findDev(1);
  }
  if (dev.empty()) {
    std::cerr << "cannot find motor device" << std::endl;
    exit(-1);
  };

  Motor::RMDS m(dev, 1);

  std::vector<double> params;
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
      while (i < argc) {
        args.push_back(std::string(argv[i]));
        i++;
      }
    }
  }

  if (args[0] == "-t") {
    if (args.size() == 2)
      m.rotateTo(std::stod(args[1]));
    else
      m.rotateTo(std::stod(args[1]), std::stod(args[2]));
  } else if (args[0] == "-m") {
    if (args.size() == 2)
      m.rotateMore(std::stod(args[1]));
    else
      m.rotateMore(std::stod(args[1]), std::stod(args[2]));
  } else if (args[0] == "-r") {
    if (args.size() == 2)
      m.rotate(std::stod(args[1]));
    else
      m.rotate();
    std::cout << "run" << std::endl;
  } else if (args[0] == "-p") {
    m.pause();
  } else if (args[0] == "-g") {
    // auto start = std::chrono::system_clock::now();
    // for (size_t i = 0; i < 1000; i++)
    // {
    //    std::cout << m.getCurrentPose() << std::endl;
    // }
    // auto end = std::chrono::system_clock::now();
    // auto time = std::chrono::duration<double>(end - start).count();
    // std::cout << "total cost: " << time << std::endl;
    // std::cout << "echo freq: " << 1000 / time << "hz" << std::endl;
    std::cout << m.getCurrentPose() << std::endl;
  } else if (args[0] == "-mt") {
    if (args.size() == 2)
      m.rotateMTo(std::stod(args[1]));
    else
      m.rotateMTo(std::stod(args[1]), std::stod(args[2]));
  } else if (args[0] == "-mm") {
    if (args.size() == 2)
      m.rotateMMore(std::stod(args[1]));
    else
      m.rotateMMore(std::stod(args[1]), std::stod(args[2]));
  } else if (args[0] == "-mg") {
    std::cout << m.getCurrentMPose() << std::endl;
  }

  return 0;
}
