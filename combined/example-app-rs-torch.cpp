#include <torch/script.h> // One-stop header.

#include <iostream>
#include <memory>
#include<chrono>
#include <librealsense2/rs.hpp>
#include<thread>


void thread_fn_1() {
  try {
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();
    std::cout << "Pipeline started\n";

    while (true) {
      rs2::frameset frames = p.wait_for_frames();

      // Try to get a frame of a depth image
      rs2::depth_frame depth = frames.get_depth_frame();

      // Get the depth frame's dimensions
      float width = depth.get_width();
      float height = depth.get_height();

      // Query the distance from the camera to the object in the center of the image
      float dist_to_center = depth.get_distance(width / 2, height / 2);

      // Print the distance 
      std::cout << "The camera is facing an object " << dist_to_center << " meters away         \r";
  }

  }
  catch (const rs2::error & e)
  {
      std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
      return;
  }
  catch (const std::exception & e)
  {
      std::cerr << e.what() << std::endl;
      return;
  }
}



void thread_fn_2() {
  std::cout << "Inside thread\n";
  try {
    torch::jit::script::Module module;
    std::cout << "Inside thread 2\n";


    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load("/home/julius/robotics/rs-torch-panda/torch/traced_policy.pt");
    std::cout << "Loaded model successfully\n";
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::zeros({1, 3*84*84}));
    at::Tensor pred = module.forward(inputs).toTensor();
    std::cout << pred << '\n';
    float predictions = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    while (true) {
      std::vector<torch::jit::IValue> inputs;
      inputs.push_back(torch::zeros({1, 3*84*84}));
      at::Tensor pred = module.forward(inputs).toTensor();
      predictions += 1;
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      float duration = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
      if (duration > 0) {
        float freq = predictions / (duration/1000);
        std::cout << "Frequency = " << freq << " Preds = " << predictions << " Dur = " << duration << "\r";
      }
      

    }
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return;
  } 

}



int main(int argc, const char* argv[]) {
  if (argc != 2) {
    std::cerr << "usage: example-app <path-to-exported-script-module>\n";
    return -1;
  }


  std::cout << "The heööd\n";


  
  try {
    

    /*
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();
    std::cout << "Pipeline started\n";

    while (false) {
      rs2::frameset frames = p.wait_for_frames();

      // Try to get a frame of a depth image
      rs2::depth_frame depth = frames.get_depth_frame();

      // Get the depth frame's dimensions
      float width = depth.get_width();
      float height = depth.get_height();

      // Query the distance from the camera to the object in the center of the image
      float dist_to_center = depth.get_distance(width / 2, height / 2);

      // Print the distance 
      std::cout << "The camera is facing an object " << dist_to_center << " meters away         \r";
    }*/
  
    //std::thread t1(thread_fn_1);
    //std::thread t2(thread_fn_2);

    //t1.join();
    //t2.join();
    

    
    torch::jit::script::Module module;


    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(argv[1]);
    std::cout << "Loaded model successfully\n";
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::zeros({1, 3*84*84}));
    at::Tensor pred = module.forward(inputs).toTensor();
    std::cout << pred << '\n';
    float predictions = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    while (true) {
      std::vector<torch::jit::IValue> inputs;
      inputs.push_back(torch::zeros({1, 3*84*84}));
      at::Tensor pred = module.forward(inputs).toTensor();
      predictions += 1;
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      float duration = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
      if (duration > 0) {
        float freq = predictions / (duration/1000);
        std::cout << "Frequency = " << freq << " Preds = " << predictions << " Dur = " << duration << "\r";
      }
      

    }

    return EXIT_SUCCESS; 

  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return -1;
  } 
}