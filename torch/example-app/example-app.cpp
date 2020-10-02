#include <torch/script.h> // One-stop header.

#include <iostream>
#include <memory>
#include<chrono>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

int main(int argc, const char* argv[]) {
  if (argc != 2) {
    std::cerr << "usage: example-app <path-to-exported-script-module>\n";
    return -1;
  }


  torch::jit::script::Module module;
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(argv[1]);
    std::cout << "Loaded model successfully\n";
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::zeros({1, 3*84*84}));
    at::Tensor pred = module.forward(inputs).toTensor();
    std::cout << pred << '\n';
    float predictions = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    pipe.start();

    while (true) {
      std::vector<torch::jit::IValue> inputs;
      inputs.push_back(torch::zeros({1, 3*84*84}));
      at::Tensor pred = module.forward(inputs).toTensor();
      predictions += 1;
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      float duration = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
      if (duration > 0) {
        float freq = predictions / (duration/1000);
        std::cout << "Frequency = " << freq << " Preds = " << predictions << " Dur = " << duration << std::endl;
      }
      
      // std::cout << pred << '\n';

    }

  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return -1;
  }

  std::cout << "ok\n";
}