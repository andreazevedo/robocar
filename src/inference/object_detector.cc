#include "inference/object_detector.h"

#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

#include "inference/object.h"

namespace robocar {
namespace inference {

namespace {

constexpr float kImageMean = 128.0f;
constexpr float kImageStd = 128.0f;

ObjectLocation getObjectLocation(const float* location) {
  return ObjectLocation{location[0], location[1], location[2], location[3]};
}

}  // namespace

ObjectDetector::ObjectDetector(const std::string& modelFile, bool isQuantized,
                               std::vector<std::string> labels)
    : labels_(std::move(labels)) {
  // Load model.
  model_ = tflite::FlatBufferModel::BuildFromFile(modelFile.c_str());
  if (!model_) {
    throw std::runtime_error("Failed to load model " + modelFile);
  }

  // Create interpreter.
  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
  if (!interpreter_) {
    throw std::runtime_error("Failed to create interpreter");
  }
  if (interpreter_->AllocateTensors() != kTfLiteOk) {
    throw std::runtime_error("Failed to allocate tensors");
  }
  interpreter_->SetNumThreads(1);

  // Find input tensors.
  if (interpreter_->inputs().size() != 1) {
    throw std::runtime_error("Graph needs to have just 1 input");
  }
  inputTensor_ = interpreter_->tensor(interpreter_->inputs()[0]);
  if (isQuantized) {
    if (inputTensor_->type != kTfLiteUInt8) {
      throw std::runtime_error(
          "Quantized graph's input should be kTfLiteUInt8");
    }
  } else {
    if (inputTensor_->type != kTfLiteFloat32) {
      throw std::runtime_error(
          "Non-quantized graph's input should be kTfLiteFloat32");
    }
  }

  // Find output tensors.
  if (interpreter_->outputs().size() != 4) {
    throw std::runtime_error("Graph needs to have 4 and only 4 outputs");
  }
}

std::vector<DetectedObject> ObjectDetector::runDetection(const cv::Mat& image,
                                                         cv::Mat* outputImage) {
  if (!image.data) {
    throw std::runtime_error("Image is empty!");
  }
  if (width() != image.cols || height() != image.rows) {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(width(), height()));
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    feedInImage(resized);
  } else {
    cv::Mat forTf;
    cv::cvtColor(image, forTf, cv::COLOR_BGR2RGB);
    feedInImage(forTf);
  }

  if (interpreter_->Invoke() != kTfLiteOk) {
    throw std::runtime_error("Error invoking interpreter");
  }

  const float* locations = interpreter_->typed_output_tensor<float>(0);
  const float* classes = interpreter_->typed_output_tensor<float>(1);
  const float* scores = interpreter_->typed_output_tensor<float>(2);
  const float* numDetections = interpreter_->typed_output_tensor<float>(3);

  std::vector<DetectedObject> objects;
  for (int i = 0; i < *numDetections; ++i) {
    int classId = static_cast<int>(classes[i]);
    objects.push_back(DetectedObject{classId, getLabel(classId), scores[i],
                                     getObjectLocation(locations + (i * 4))});
  }

  if (outputImage) {
    *outputImage = image;
    annotateImage(*outputImage, objects);
  }

  return objects;
}

std::string ObjectDetector::getLabel(size_t index) const {
  if (index >= labels_.size()) {
    return "<out of range>";
  }
  return labels_[index];
}

void ObjectDetector::feedInImage(const cv::Mat& image) {
  switch (inputTensor_->type) {
    case kTfLiteFloat32: {
      float* dst = interpreter_->typed_input_tensor<float>(0);
      const int rowElements = width() * inputChannels();
      for (int row = 0; row < height(); row++) {
        const uchar* rowPtr = image.ptr(row);
        for (int i = 0; i < rowElements; i++) {
          dst[i] = (rowPtr[i] - kImageMean) / kImageStd;
        }
        dst += rowElements;
      }
    } break;
    case kTfLiteUInt8: {
      uchar* dst = interpreter_->typed_input_tensor<uchar>(0);
      const int rowElements = width() * inputChannels();
      for (int row = 0; row < height(); row++) {
        ::memcpy(dst, image.ptr(row), rowElements);
        dst += rowElements;
      }
    } break;
    default:
      throw std::runtime_error("Unexpected input tensor type");
  }
}

void ObjectDetector::annotateImage(cv::Mat& image,
                                   const std::vector<DetectedObject>& objects) {
  for (const auto& object : objects) {
    const int ymin = object.location.top * image.rows;
    const int xmin = object.location.left * image.cols;
    const int ymax = object.location.bottom * image.rows;
    const int xmax = object.location.right * image.cols;
    if (object.score > 0.6f) {
      cv::rectangle(image, cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin),
                    cv::Scalar(0, 0, 255), 1);
      cv::putText(image, object.className, cv::Point(xmin, ymin - 5),
                  cv::FONT_HERSHEY_COMPLEX, .8, cv::Scalar(10, 255, 30));
    }
  }
}

}  // namespace inference
}  // namespace robocar

