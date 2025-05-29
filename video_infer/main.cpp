/**
 * YOLOv8 Human Detection via TensorRT Inference.
 * 
 * This is a lightweight adaptation for Jetson Xavier based on:
 * https://github.com/Qengineering/YoloV8-TensorRT-Jetson_Nano
 *
 * This program loads a YOLOv8 TensorRT engine and performs real-time human detection
 * from an IP camera stream. It identifies the largest detected bounding box
 * (likely the closest person) and sends its coordinates via UDP to a local server.
 *
 * - Author: <Your Name>
 * - Subject: CSSE4011 Embedded Systems
 * - Compile with: g++ -std=c++17 -O2 main.cpp -o detector `pkg-config --cflags --libs opencv4` -lnvinfer -lcudart
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <chrono>
#include "NvInferPlugin.h"
#include "common.hpp"
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

static Logger gLogger;

// Structure to hold detection results
struct Detection {
    cv::Rect box;
    float confidence;
};

// Struct to hold image preprocessing parameters
struct Param {
    float ratio, dw, dh, width, height;
};

// YOLO model parameters
cv::Size       im_size(640, 640);
const int      num_labels  = 80;
const int      topk        = 100;
const float    score_thres = 0.45f;
const float    iou_thres   = 0.05f;

class YOLOv8Detector {
public:
    // Constructor: Load engine, prepare execution context, and initialize UDP socket
    YOLOv8Detector(const std::string& engine_path) {
        std::ifstream file(engine_path, std::ios::binary);
        if (!file.good()) throw std::runtime_error("Failed to open engine file");

        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> engine_data(size);
        file.read(engine_data.data(), size);
        file.close();

        initLibNvInferPlugins(&gLogger, "");
        nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
        engine = runtime->deserializeCudaEngine(engine_data.data(), size);
        context = engine->createExecutionContext();
        if (!engine || !context) throw std::runtime_error("Engine creation failed");

        // Identify input/output bindings
        for (int i = 0; i < engine->getNbBindings(); ++i) {
            if (engine->bindingIsInput(i)) inputIndex = i;
            else outputIndex = i;
        }

        inputDims = engine->getBindingDimensions(inputIndex);
        inputH = inputDims.d[2];
        inputW = inputDims.d[3];

        outputDims = engine->getBindingDimensions(outputIndex);
        numElements = 1;
        for (int i = 0; i < outputDims.nbDims; ++i) numElements *= outputDims.d[i];

        // Allocate device buffers and CUDA stream
        CHECK(cudaMalloc(&buffers[inputIndex], getSizeByDims(inputDims) * sizeof(float)));
        CHECK(cudaMalloc(&buffers[outputIndex], numElements * sizeof(float)));
        CHECK(cudaStreamCreate(&stream));

        // UDP socket for sending bounding box coordinates
        this->sock = socket(AF_INET, SOCK_DGRAM, 0);
        this->server_addr = {};
        this->server_addr.sin_family = AF_INET;
        this->server_addr.sin_port = htons(9999);
        inet_pton(AF_INET, "127.0.0.1", &this->server_addr.sin_addr);
    }

    ~YOLOv8Detector() {
        cudaFree(buffers[inputIndex]);
        cudaFree(buffers[outputIndex]);
        cudaStreamDestroy(stream);
        if (context) context->destroy();
        if (engine) engine->destroy();
        close(this->sock);
    }

    // Run inference on a single frame
    std::vector<Detection> infer(const cv::Mat& image) {
        std::vector<float> inputTensor;
        preprocess(image, inputTensor);

        context->setBindingDimensions(inputIndex, nvinfer1::Dims4{1, 3, inputH, inputW});
        if (!context->allInputDimensionsSpecified()) {
            throw std::runtime_error("Incomplete binding dimensions");
        }

        // Transfer input to GPU
        cudaMemcpyAsync(buffers[inputIndex], inputTensor.data(), inputTensor.size() * sizeof(float), cudaMemcpyHostToDevice, stream);

        // Run inference
        if (!context->enqueueV2(buffers, stream, nullptr)) {
            throw std::runtime_error("Inference failed");
        }

        // Copy results from GPU
        std::vector<float> outputTensor(numElements);
        cudaMemcpyAsync(outputTensor.data(), buffers[outputIndex], outputTensor.size() * sizeof(float), cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);

        std::vector<Detection> detections;
        postprocess(outputTensor, detections, pparam);
        return detections;
    }

private:
    // Resize and pad input image to fit model input
    void preprocess(const cv::Mat& img, std::vector<float>& inputTensor) {
        cv::Mat padded;
        Letterbox(img, padded, cv::Size(inputW, inputH), pparam);

        cv::Mat blob;
        cv::dnn::blobFromImage(padded, blob, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false, CV_32F);
        inputTensor.assign((float*)blob.datastart, (float*)blob.dataend);
    }

    // Apply padding and scaling to preserve aspect ratio
    void Letterbox(const cv::Mat& img, cv::Mat& out, const cv::Size& new_shape, Param& param) {
        float img_w = img.cols, img_h = img.rows;
        float r = std::min(new_shape.width / img_w, new_shape.height / img_h);
        int new_w = std::round(img_w * r), new_h = std::round(img_h * r);
        int dw = (new_shape.width - new_w) / 2;
        int dh = (new_shape.height - new_h) / 2;

        cv::resize(img, out, cv::Size(new_w, new_h));
        cv::copyMakeBorder(out, out, dh, dh, dw, dw, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

        param = {1.0f / r, float(dw), float(dh), img_w, img_h};
    }

    // Postprocess to find the largest valid detection and send via UDP
    void postprocess(const std::vector<float>& outputTensor, std::vector<Detection>& detections, const Param& param) {
        detections.clear();
        int num_classes = outputDims.d[1] - 4;
        int num_anchors = outputDims.d[2];
        cv::Mat output(num_classes + 4, num_anchors, CV_32F, (void*)outputTensor.data());
        output = output.t();

        float max_area = 0.0f;
        int best_index = -1;
        float best_score = 0.0f;

        for (int i = 0; i < num_anchors; ++i) {
            const float* ptr = output.ptr<float>(i);
            const float* class_ptr = ptr + 4;
            int class_id = std::max_element(class_ptr, class_ptr + num_classes) - class_ptr;
            float score = class_ptr[class_id];

            if (class_id == 0 && score > score_thres) {
                // Calculate bounding box area in original image scale
                float x = ptr[0] - param.dw, y = ptr[1] - param.dh;
                float w = ptr[2], h = ptr[3];
                float x0 = clamp((x - w / 2) * param.ratio, 0.f, param.width);
                float y0 = clamp((y - h / 2) * param.ratio, 0.f, param.height);
                float x1 = clamp((x + w / 2) * param.ratio, 0.f, param.width);
                float y1 = clamp((y + h / 2) * param.ratio, 0.f, param.height);
                float area = (x1 - x0) * (y1 - y0);
                if (area > max_area) {
                    max_area = area;
                    best_index = i;
                    best_score = score;
                }
            }
        }

        // Send largest bounding box over UDP
        if (best_index >= 0) {
            const float* ptr = output.ptr<float>(best_index);
            float x = ptr[0] - param.dw, y = ptr[1] - param.dh;
            float w = ptr[2], h = ptr[3];

            float x0 = clamp((x - w / 2) * param.ratio, 0.f, param.width);
            float y0 = clamp((y - h / 2) * param.ratio, 0.f, param.height);
            float x1 = clamp((x + w / 2) * param.ratio, 0.f, param.width);
            float y1 = clamp((y + h / 2) * param.ratio, 0.f, param.height);

            int16_t coords[4] = {
                static_cast<int16_t>(x0),
                static_cast<int16_t>(x1),
                static_cast<int16_t>(y0),
                static_cast<int16_t>(y1)
            };

            sendto(this->sock, coords, sizeof(coords), 0, (sockaddr*)&this->server_addr, sizeof(this->server_addr));

            std::cout << "[UDP] Sent: x0=" << coords[0]
                      << ", x1=" << coords[1]
                      << ", y0=" << coords[2]
                      << ", y1=" << coords[3] << std::endl;

            detections.push_back({cv::Rect(cv::Point(coords[0], coords[2]), cv::Point(coords[1], coords[3])), best_score});
        }
    }

    size_t getSizeByDims(const nvinfer1::Dims& dims) {
        size_t size = 1;
        for (int i = 0; i < dims.nbDims; ++i) size *= dims.d[i];
        return size;
    }

    // Internal variables
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t stream;
    void* buffers[2];
    int inputIndex, outputIndex, inputW, inputH, numElements;
    nvinfer1::Dims inputDims, outputDims;
    Param pparam;
    int sock;
    sockaddr_in server_addr;
};

// Application entry point
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./detector <engine.trt>" << std::endl;
        return -1;
    }

    cudaSetDevice(0);
    YOLOv8Detector detector(argv[1]);

    // Open IP camera stream
    cv::VideoCapture cap("http://192.168.0.141/stream");
    if (!cap.isOpened()) {
        std::cerr << "Camera error" << std::endl;
        return -1;
    }

    // Real-time loop
    cv::Mat frame;
    while (cap.read(frame)) {
        auto detections = detector.infer(frame);
        for (const auto& det : detections) {
            cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "Human", det.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("YOLOv8 - Human Detector", frame);
        if (cv::waitKey(1) == 27) break;  // Exit on ESC
    }

    return 0;
}
