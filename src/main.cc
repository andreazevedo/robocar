#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

#include "car.h"
#include "control/motor.h"
#include "control/util.h"
#include "inference/object_detector.h"
#include "inference/util.h"
#include "sensors/camera.h"

using robocar::control::Motor;

void testModel(robocar::sensors::Camera& camera) {
  static robocar::inference::ObjectDetector detector(
      "models/ssd_mobilenet_robocar/model.tflite", true,
      robocar::inference::loadLabels(
          "models/ssd_mobilenet_robocar/labelmap.txt"));

  const auto frame = camera.captureFrame();
  cv::Mat annotatedFrame;
  auto objects = detector.runDetection(frame, &annotatedFrame);

  for (const auto& obj : objects) {
    std::cout << obj.className << "(" << obj.classId << "): " << obj.score
              << " => [" << obj.location.top << ", " << obj.location.left
              << ", " << obj.location.bottom << ", " << obj.location.right
              << "]" << std::endl;
  }
  cv::imwrite("bin/images/annotated.jpg", annotatedFrame);
}

void takeAndSavePicture(robocar::sensors::Camera& camera, int picId) {
  static int batchId = ::time(nullptr);

  const auto frame = camera.captureFrame();
  const int width = frame.cols;
  const int height = frame.rows;

  // crop the largest possible square image (cut the left side of the pic).
  const auto cropped = frame(cv::Rect(width - height, 0, height, height));

  // resize of 300x300
  constexpr size_t kPicSize = 300;
  cv::Mat resized;
  cv::resize(cropped, resized, cv::Size(kPicSize, kPicSize));

  cv::imwrite("bin/images/train_" + std::to_string(batchId) +
                  std::to_string(picId) + ".jpg",
              resized);
}

int main(int argc, char* argv[]) {
  robocar::control::globalPigpioInitialize();

  robocar::Car car(true /* log debug info */);

  // Set terminal to raw
  // system("/bin/stty raw");
  std::atexit([]() { system("/bin/stty cooked"); });

  char c = '\0';
  while ((c = getchar()) != 'q') {
    switch (c) {
      case 'r':
        car.controlService().setMotorState(Motor::State::Backward);
        break;
      case 'f':
        car.controlService().setMotorState(Motor::State::Forward);
        break;
      case 'b':
        car.controlService().setMotorState(Motor::State::Release);
        break;
      case 'w':
        car.controlService().adjustVehicleState(+0.1 /* throttle */,
                                                0.0 /* steering */);
        break;
      case 's':
        car.controlService().adjustVehicleState(-0.1 /* throttle */,
                                                0.0 /* steering */);
        break;
      case 'a':
        car.controlService().adjustVehicleState(0.0 /* throttle */,
                                                -10.0 /* steering */);
        break;
      case 'd':
        car.controlService().adjustVehicleState(0.0 /* throttle */,
                                                +10.0 /* steering */);
        break;
      case 'p':
        // system("/bin/stty cooked");
        std::cout << '\n';
        std::cout << "Throttle: " << car.controlService().throttle()
                  << " | Steering angle: "
                  << car.controlService().steeringAngle() << " | State: "
                  << Motor::getStateString(car.controlService().state())
                  << std::endl;
        // system("/bin/stty raw");
        break;
      case 'c': {
        auto frame = car.camera().captureFrame();
        car.perceptionService().laneDetector().setSaveDebugImages(true);
        car.perceptionService().laneDetector().getLane(frame);
        cv::imwrite("bin/images/photo.jpg", frame);
        car.perceptionService().laneDetector().setSaveDebugImages(false);
      } break;
      case 'm':
        car.startAutonomyLoop();
        break;
      case 'n':
        car.stopAutonomyLoop();
        break;
      case 'o':
        car.perceptionService().laneDetector().setSaveDebugImages(true);
        car.loopOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        car.controlService().setThrottle(0.0);
        car.perceptionService().laneDetector().setSaveDebugImages(false);
        break;
      case 't':
        testModel(car.camera());
        break;
      case 'j':
        static int picId = 0;
        takeAndSavePicture(car.camera(), ++picId);
        break;
    }
  }
}
