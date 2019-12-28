#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/matx.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

namespace robocar {
namespace inference {

/**
 * Represents the position of an object in the image.
 * The positions are floating-point numbers in the [0,1] range representing
 * the location of the object in the input image.
 */
struct ObjectLocation {
  float top{0.0};
  float left{0.0};
  float bottom{0.0};
  float right{0.0};
};

/**
 * Represents an object detected by the CNN.
 */
struct DetectedObject {
  int classId{0};
  std::string className;
  float score{0.0};
  ObjectLocation location;
};

/**
 * Object detector
 */
class ObjectDetector {
 public:
  /**
   * Creates an object detector.
   *
   * @param modelFile       The path to the tflite model.
   * @param isQuantized     Whether the model is quantized.
   * @param labels          The labels of the objects.
   */
  ObjectDetector(const std::string& modelFile, bool isQuantized,
                 std::vector<std::string> labels);

  /**
   * Detect the objects.
   *
   * @param image         The input image. If the input image is not of the
   *                      size expected by the model, it will be resized.
   * @param outputImage   If not NULL, will be filled with an image annotated
   *                      with the detected objects.
   */
  std::vector<DetectedObject> runDetection(const cv::Mat& image,
                                           cv::Mat* outputImage = nullptr);

 private:
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
  const std::vector<std::string> labels_;
  const bool saveDebugImage_{false};
  TfLiteTensor* inputTensor_{nullptr};

  int width() const { return inputTensor_->dims->data[2]; }
  int height() const { return inputTensor_->dims->data[1]; }
  int inputChannels() const { return inputTensor_->dims->data[3]; }

  // feeds the image into the CNN
  void feedInImage(const cv::Mat& image);

  // gets a label for a class.
  std::string getLabel(size_t index) const;

  // annotates the image with the found objects.
  void annotateImage(cv::Mat& image,
                     const std::vector<DetectedObject>& objects);
};

}  // namespace inference
}  // namespace robocar
