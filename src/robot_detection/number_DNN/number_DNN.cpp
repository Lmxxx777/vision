#include "number_DNN.h"

using namespace cv;

namespace robot_detection {

    DNN_detect::DNN_detect()
    {
        // std::string package_path = ros::package::getPath("robot_detection");
        // std::string package_path = "/home/lmx2/vision_ws_2/src/robot_detection";
        cv::FileStorage fs("/home/lmx2/vision_ws_2/src/robot_detection/vision_data/dnn_data.yaml", FileStorage::READ);
        net_path = (std::string)fs["net_path"];
        input_width = (int)fs["input_width"];
        input_height = (int)fs["input_height"];
        net = dnn::readNetFromONNX(net_path);
    //    net.setPreferableTarget(dnn::dnn4_v20211004::DNN_TARGET_CUDA_FP16);
    //    net.setPreferableBackend(dnn::dnn4_v20211004::DNN_BACKEND_CUDA);
        fs.release();
    }

    void DNN_detect::img_processing(Mat ori_img, std::vector<cv::Mat>& numROIs) {
        Mat out_blob;
        cvtColor(ori_img, ori_img, cv::COLOR_RGB2GRAY);
        threshold(ori_img, ori_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // cv::resize(ori_img,ori_img,Size(input_width,input_height));
        numROIs.push_back(ori_img);
    }

    Mat DNN_detect::net_forward(const std::vector<cv::Mat>& numROIs) {
        //!< opencv dnn supports dynamic batch_size, later modify
        cv::Mat blob;
        dnn::blobFromImages(numROIs, blob, 1.0f/255.0f, Size(input_width, input_height));
        net.setInput(blob);
        Mat outputs = net.forward();
        return outputs;
    }

}
