//
// Created by sdhm on 3/21/19.
// 获取kinect2的深度信息
//

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <pickup_msg/PickupObj.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#if defined(__linux__)
#include <sys/prctl.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

using namespace cv;
using namespace dnn;
using namespace std;

class Receiver
{
public:
    enum Mode
    {
        IMAGE = 0,
        CLOUD,
        BOTH
    };

private:
    std::mutex lock;

    const std::string topicColor, topicDepth;
    const bool useExact, useCompressed, showImage;

    bool updateImage, updateCloud;
    bool save;
    bool running;
    size_t frame;
    const size_t queueSize;

    cv::Mat color, depth;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

    std::thread imageViewerThread, tfPublisher;
    Mode mode;

    ros::Subscriber subObjMsg;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PCDWriter writer;
    std::ostringstream oss;
    std::vector<int> params;

    tf::TransformBroadcaster broadcaster;
    // 待处理物体位置及标签信息
    std::vector<cv::Rect2f> boxes;
    std::vector<uint32_t> labels;
    string classes[80]={"person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign",
                          "parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
                          "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard",
                          "surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
                          "broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa","pottedplant","bed","diningtable","toilet","tvmonitor",
                          "laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase",
                          "scissors","teddy bear","hair drier","toothbrush"};

public:
    Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed, const bool showImage)
            : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed), showImage(showImage),
              updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5), it(nh), mode(CLOUD)
    {
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(100);
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(1);
        params.push_back(cv::IMWRITE_PNG_STRATEGY);
        params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
        params.push_back(0);

        subObjMsg = nh.subscribe("PickupObjMsg", 1, &Receiver::objMsgCallback, this);
    }

    ~Receiver()
    {
    }

    void run(const Mode mode)
    {
        start(mode);
        stop();
    }

private:

    void start(const Mode mode)
    {
        this->mode = mode;
        running = true;

        std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
        std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
        subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
        subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

        if(useExact)
        {
            syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
        }
        else
        {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
        }

        std::chrono::milliseconds duration(1);
        while(!updateImage || !updateCloud)
        {
            if(!ros::ok())
            {
                return;
            }
            std::this_thread::sleep_for(duration);
        }
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud->height = color.rows;
        cloud->width = color.cols;
        cloud->is_dense = false;
        cloud->points.resize(cloud->height * cloud->width);
        createLookup(this->color.cols, this->color.rows);

        // 开线程实时发布物体pose
//        tfPublisher = std::thread(&Receiver::publishTF, this);

        switch(mode)
        {
            case CLOUD:
                cloudViewer();
                break;
            case IMAGE:
//                imageViewer();
                imageDetector();
                break;
            case BOTH:
//                imageViewerThread = std::thread(&Receiver::imageViewer, this);
//                imageViewerThread = std::thread(&Receiver::imageDetector, this);
                cloudViewer();
                break;
        }
    }

    void stop()
    {
//        spinner.stop();

        if(useExact)
        {
            delete syncExact;
        }
        else
        {
            delete syncApproximate;
        }

        delete subImageColor;
        delete subImageDepth;
        delete subCameraInfoColor;
        delete subCameraInfoDepth;

        running = false;
        if(mode == BOTH)
        {
            imageViewerThread.join();
        }
    }

    void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                  const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
    {
        cv::Mat color, depth;

        readCameraInfo(cameraInfoColor, cameraMatrixColor);
        readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
        readImage(imageColor, color);
        readImage(imageDepth, depth);

        // IR image input
        if(color.type() == CV_16U)
        {
            cv::Mat tmp;
            color.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, color, CV_GRAY2BGR);
        }

        lock.lock();
        this->color = color;
        this->depth = depth;
        updateImage = true;
        updateCloud = true;
        lock.unlock();
    }

    void objMsgCallback(const pickup_msg::PickupObj::ConstPtr& msg)
    {
        // 打印物体位置及标签信息
//        for (size_t i = 0; i < msg->boxes.size(); ++i) printf("%f ", msg->boxes[i]);
//        printf("\n");
//        for (size_t i = 0; i < msg->labels.size(); ++i) printf("%d ", msg->labels[i]);

        // 预处理处理边界框
        std::vector<cv::Rect2f> _boxes;
        uint32_t boxesNum = msg->boxes.size()/4;
        for (size_t i = 0; i < boxesNum; ++i){
            _boxes.push_back(cv::Rect2f( cv::Point2f(msg->boxes[4*i], msg->boxes[4*i+1]), cv::Point2f(msg->boxes[4*i+2], msg->boxes[4*i+3])));
        }

        // 验证数据可靠性
        if(boxesNum == msg->labels.size()) {
            // 存储物体位置及标签信息
            boxes = _boxes;
            labels = msg->labels;
//            printf("\ndetected %d objects\n", boxesNum);
        } else printf("Some errors occur in the transmission of object information...\n");
    }

    void publishTF()
    {
        tf::TransformBroadcaster broadcaster;
        tf::Transform objTf;

        geometry_msgs::PoseStamped recognized_object_pose;
        recognized_object_pose.header.frame_id = "recognized_object";

        for(; running && ros::ok();)
        {
            recognized_object_pose.header.stamp = ros::Time();

//            recognized_object_pose.pose.position.x = objPose[0];
//            recognized_object_pose.pose.position.y = objPose[1];
//            recognized_object_pose.pose.position.z = objPose[2];
//            recognized_object_pose.pose.orientation.w = 1;
//
//            printf("recognized_object_pose:%f %f %f\n", objPose[0], objPose[1], objPose[2]);

//            tf::Quaternion qPose;
//            qPose.setRPY(0, 0, 0);

            objTf = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(recognized_object_pose.pose.position.x,
                                                     recognized_object_pose.pose.position.y, recognized_object_pose.pose.position.z));

            broadcaster.sendTransform(tf::StampedTransform(objTf, ros::Time::now(), "kinect2_rgb_optical_frame", "recognized_object"));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // 像素坐标系中的点转换到相机坐标系, 传入行,列，对应像素的y,x
    cv::Scalar pixelToCameraSpace(int row, int col){
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

        const uint16_t *itD = depth.ptr<uint16_t>(row) + col;
        const float y = lookupY.at<float>(0, row);
        const float *itX = lookupX.ptr<float>() + col;
        register const float depthValue = *itD / 1000.0f;

        float cameraSpaceX, cameraSpaceY; // 相机坐标系中的坐标
        if (*itD == 0){
            cameraSpaceX = cameraSpaceY = badPoint;
        }
        else {
            cameraSpaceX = *itX * depthValue;
            cameraSpaceY = y * depthValue;
        }

//        printf("Point[%d, %d](x,y,z):(%f %f %f)\n", row, col, cameraSpaceX, cameraSpaceY, depthValue);

        return cv::Scalar(cameraSpaceX, cameraSpaceY, depthValue);


//        for(int r = 0; r < depth.rows; ++r)
//        {
//            const uint16_t *itD = depth.ptr<uint16_t>(r);
//            const float y = lookupY.at<float>(0, r);
//            const float *itX = lookupX.ptr<float>();
//
//            for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itD, ++itX)
//            {
//                register const float depthValue = *itD / 1000.0f;
//
//                if (r==382 && c==280) printf("point[382, 280](x,y,z):(%f %f %f)\n", *itX * depthValue, y * depthValue, depthValue);
//            }
//        }
    }

    // 获取目标中心点的深度信息
    cv::Scalar getObjPose(int bottleNum, int x, int y, int width, int height, Mat& frame, Mat& depth){
        int centerX = x + width/2; // 中心点像素x坐标
        int centerY = y + height/2; // 中心点像素y坐标
        std::ostringstream oss; // 输出流

        float avgDepth = 0; // 平均深度值
        float avgX = 0, avgY = 0; // 相机坐标系下的坐标, 单位:m

        // 打印中心点位置及深度信息
//        printf("bottle%dCenter:[%d,%d] ", bottleNum, centerX, centerY);
//        printf("bottle%dcenterDepth[%d %d]:%f ", bottleNum, centerY, centerX, depth.at<uint16_t>(centerY, centerX) / 1000.0f);

        // 获取中心点像机坐标系中的坐标
//        Scalar point = pixelToCameraSpace(centerY, centerX);
//        printf("bottle%dCenterPose(X Y Z):(%f %f %f)\n", bottleNum, point[0], point[1], point[2]);

        int calPointNum = 0; // 用于计算pose的点数
        // 深度信息的 列对应x, 行对应y
        for (int col = centerX -2;col < centerX +2;col++) {
            for (int row = centerY -2; row < centerY +2; row++) {

                float depthValue = depth.at<uint16_t>(row,col) / 1000.0f;
//                printf("depth[%d %d]:%f  ", row, col, depthValue);

                if(depthValue > 0.01 && depthValue < 2.0){ // 确保有深度信息(有深度信息才能计算相机坐标系下的坐标)
                    calPointNum++;
                    avgDepth += depthValue;

                    Scalar point = pixelToCameraSpace(row, col); // 获取像机坐标系中的坐标
                    avgX += point[0];
                    avgY += point[1];
                }
            }
//            printf("\n");
        }
        avgDepth /= calPointNum;
        avgX /= calPointNum;
        avgY /= calPointNum;

        // 发布TF及物体坐标消息
        string bottleName = "bottle" + to_string(bottleNum);
        if (avgX > -5.0 && avgX < 5.0 && avgY > -5.0 && avgY < 5.0 && avgDepth > 0.1 && avgDepth < 2.0) {
            tf::Transform objTf = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(avgX, avgY, avgDepth));
            broadcaster.sendTransform(tf::StampedTransform(objTf, ros::Time::now(), "kinect2_rgb_optical_frame",
                                                           bottleName));
        }

        // 打印有效点的个数及对应相机坐标系中的平均pose
//        printf("calPointNum:%d avgX:%f avgY:%f avgDepth:%f ", calPointNum, avgX, avgY, avgDepth);
        printf("bottle%dAvgPose(X Y Z):(%f %f %f)\n\n", bottleNum, avgX, avgY, avgDepth);
        // 图像中显示各瓶子空间位置
        if(showImage) {
            rectangle(frame, Point(centerX -2, centerY -2), Point(centerX +2, centerY +2), Scalar(0, 0, 255), FILLED);
            oss << bottleName << "Pose: (" << avgX << " " << avgY << " " << avgDepth << ")";
            cv::putText(frame, oss.str(), cv::Point(250, 15 * bottleNum), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        CV_RGB(255, 0, 0), 1, CV_AA);
        }

        return cv::Scalar(avgX, avgY, avgDepth);
    }

    // Draw the predicted bounding box
    void drawPred(int classId, int left, int top, int right, int bottom, Mat& frame)
    {
        //Draw a rectangle displaying the bounding box
        rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 2);

        //Get the label for the class name and its confidence
        string label = classes[classId];

        //Display the label at the top of the bounding box
        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        rectangle(frame, Point(left-1, top - round(1.2*labelSize.height)), Point(left + round(1.2*labelSize.width), top + baseLine), Scalar(255, 178, 50), FILLED);
        putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0),1,CV_AA);
    }

    // Remove the bounding boxes with low confidence using non-maxima suppression
    void postprocess(Mat& frame, Mat& depth)
    {
        int bottleNum = 0; // 瓶子个数
        for (size_t i = 0; i < boxes.size(); ++i){
            if(showImage) drawPred(labels[i], boxes[i].x, boxes[i].y, boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height, frame);

            // 计算各瓶子空间位置
            if (classes[labels[i]] == "bottle") {
                bottleNum ++;
                getObjPose(bottleNum, boxes[i].x, boxes[i].y, boxes[i].width, boxes[i].height, frame, depth);
            }
        }
    }

    void imageDetector()
    {
        cv::Mat color, depth, depthDisp, combined, depthm;
        std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
        double fps = 0;
        size_t frameCount = 0;
        std::ostringstream oss;

        cv::Mat blob;
        start = std::chrono::high_resolution_clock::now();
        for(; running && ros::ok();){

            if(updateImage){
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateImage = false;
                lock.unlock();

                ++frameCount;
                now = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
                if(elapsed >= 1.0){
                    fps = frameCount / elapsed;
                    oss.str("");
                    oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
                    cout << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)" << endl << endl;
                    start = now;
                    frameCount = 0;
                }

                getDepth(depth, depthm, 1000.0f);

                /************************************ 图像处理 ************************************/
                if(!boxes.empty() && !labels.empty()) { // 接收到物体信息后才进行处理
                    postprocess(color, depth);
//                    boxes.clear();
//                    labels.clear();
//                    printf("deal img\n");
                }

                // 画中心区域框
                if(showImage) {
                    cv::rectangle(color, cv::Point(depth.cols/2-2,depth.rows/2-2), cv::Point(depth.cols/2+2,depth.rows/2+2), cv::Scalar(0,0,255), 1);
                    cv::putText(color, oss.str(), cv::Point(5, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1, CV_AA);
                    cv::imshow("object detection yolo", color);

//                    dispDepth(depth, depthDisp, 12000.0f);
//                    cv::imshow("depthDisp", depthDisp);
                }
            }

            int key = cv::waitKey(1);
            switch(key & 0xFF)
            {
                case 27:
                case 'q':
                    running = false;
                    break;
                case ' ':
                case 's':
                    if(mode == IMAGE){
                        createCloud(depth, color, cloud);
//                        saveCloudAndImages(cloud, color, depth, depthDisp);
                    }
                    else{
                        save = true;
                    }
                    break;
            }
        }
        cv::destroyAllWindows();
        cv::waitKey(100);
    }

    void imageViewer()
    {
        cv::Mat color, depth, depthDisp, combined, depthm;
        std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
        double fps = 0;
        size_t frameCount = 0;
        std::ostringstream oss;
        const cv::Point pos(5, 15);
        const cv::Scalar colorText = CV_RGB(255, 255, 255);
        const double sizeText = 0.5;
        const int lineText = 1;
        const int font = cv::FONT_HERSHEY_SIMPLEX;

        cv::namedWindow("Image Viewer");
        oss << "starting...";

        start = std::chrono::high_resolution_clock::now();
        for(; running && ros::ok();)
        {
            if(updateImage)
            {
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateImage = false;
                lock.unlock();

                cv::imshow("color", color);

                ++frameCount;
                now = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
                if(elapsed >= 1.0)
                {
                    fps = frameCount / elapsed;
                    oss.str("");
                    oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
                    start = now;
                    frameCount = 0;
                }

                dispDepth(depth, depthDisp, 12000.0f);
                combine(color, depthDisp, combined);
//                combined = color;

                // 打印中心区域深度信息
//                for(int row = depth.rows/2-2;row <= depth.rows/2+2;row++) {
//                    for (int col = depth.cols/2-2; col <= depth.cols/2+2; col++) {
//                        printf("depth[%d %d]:%f  ", row, col, depth.at<uint16_t>(row,col) / 1000.0f);
//                    }
//                    printf("\n");
//                }
//                printf("\n");

//                cout << "depth:" << depth.at<uint16_t>(212,256)/1000.0f << endl;
//
//                         for(int i=0;i<depth.rows;i++)
//                         printf("%d    ", depth.rows-1);
//
//                for(int i=depth.rows-1;i<depth.rows;i++)
//                {
//                  for(int j=0;j<depth.cols;j++)
//                    cout << depth.at<uint16_t>(i,j)/1000.0f <<' ';
//                  cout << endl;
//                }
//                cout << endl << endl;

                // 画中心区域框
                cv::rectangle(combined, cv::Point(depth.cols/2-2,depth.rows/2-2), cv::Point(depth.cols/2+2,depth.rows/2+2), cv::Scalar(0,0,255), 1);

                cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
                cv::imshow("Image Viewer", combined);
//                cv::imshow("depthDisp", depthDisp);
//                cv::imshow("depth", depth);
            }

            int key = cv::waitKey(1);
            switch(key & 0xFF)
            {
                case 27:
                case 'q':
                    running = false;
                    break;
                case ' ':
                case 's':
                    if(mode == IMAGE)
                    {
                        createCloud(depth, color, cloud);
                        saveCloudAndImages(cloud, color, depth, depthDisp);
                    }
                    else
                    {
                        save = true;
                    }
                    break;
            }
        }
        cv::destroyAllWindows();
        cv::waitKey(100);
    }

    void getDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
    {
        cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
        const uint32_t maxInt = 255;

#pragma omp parallel for
        for(int r = 0; r < in.rows; ++r)
        {
            const uint16_t *itI = in.ptr<uint16_t>(r);
            uint8_t *itO = tmp.ptr<uint8_t>(r);

            for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
            {
                *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
            }
        }
        cv::imshow("tmp", tmp);
    }

    void cloudViewer()
    {
        cv::Mat color, depth;
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
        const std::string cloudName = "rendered";

        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        visualizer->addPointCloud(cloud, cloudName);
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(0, 0, 0);
        visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
        visualizer->setSize(color.cols, color.rows);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

        for(; running && ros::ok();)
        {
            if(updateCloud)
            {
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateCloud = false;
                lock.unlock();

                createCloud(depth, color, cloud);

                visualizer->updatePointCloud(cloud, cloudName);
            }
            if(save)
            {
                save = false;
                cv::Mat depthDisp;
                dispDepth(depth, depthDisp, 12000.0f);
                saveCloudAndImages(cloud, color, depth, depthDisp);
            }
            visualizer->spinOnce(10);
        }
        visualizer->close();
    }

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
    {
        if(event.keyUp())
        {
            switch(event.getKeyCode())
            {
                case 27:
                case 'q':
                    running = false;
                    break;
                case ' ':
                case 's':
                    save = true;
                    break;
            }
        }
    }

    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        pCvImage->image.copyTo(image);
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
    {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for(size_t i = 0; i < 9; ++i, ++itC)
        {
            *itC = cameraInfo->K[i];
        }
    }

    void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
    {
        cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
        const uint32_t maxInt = 255;

#pragma omp parallel for
        for(int r = 0; r < in.rows; ++r)
        {
            const uint16_t *itI = in.ptr<uint16_t>(r);
            uint8_t *itO = tmp.ptr<uint8_t>(r);

            for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
            {
                *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
            }
        }

        cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
    }

    void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
    {
        out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

#pragma omp parallel for
        for(int r = 0; r < inC.rows; ++r)
        {
            const cv::Vec3b
                    *itC = inC.ptr<cv::Vec3b>(r),
                    *itD = inD.ptr<cv::Vec3b>(r);
            cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

            for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
            {
                itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
                itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
                itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
            }
        }
    }

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
    {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
        for(int r = 0; r < depth.rows; ++r)
        {
            pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
            const uint16_t *itD = depth.ptr<uint16_t>(r);
            const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
            const float y = lookupY.at<float>(0, r);
            const float *itX = lookupX.ptr<float>();

            for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
            {
                register const float depthValue = *itD / 1000.0f;
                // Check for invalid measurements
                if(*itD == 0)
                {
                    // not valid
                    itP->x = itP->y = itP->z = badPoint;
                    itP->rgba = 0;
                    continue;
                }
                itP->z = depthValue;
                itP->x = *itX * depthValue;
                itP->y = y * depthValue;
                itP->b = itC->val[0];
                itP->g = itC->val[1];
                itP->r = itC->val[2];
                itP->a = 255;
            }
        }
    }

    void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
    {
        oss.str("");
        oss << "./" << std::setfill('0') << std::setw(4) << frame;
        const std::string baseName = oss.str();
        const std::string cloudName = baseName + "_cloud.pcd";
        const std::string colorName = baseName + "_color.jpg";
        const std::string depthName = baseName + "_depth.png";
        const std::string depthColoredName = baseName + "_depth_colored.png";

        OUT_INFO("saving cloud: " << cloudName);
        writer.writeBinary(cloudName, *cloud);
        OUT_INFO("saving color: " << colorName);
        cv::imwrite(colorName, color, params);
        OUT_INFO("saving depth: " << depthName);
        cv::imwrite(depthName, depth, params);
        OUT_INFO("saving depth: " << depthColoredName);
        cv::imwrite(depthColoredName, depthColored, params);
        OUT_INFO("saving complete!");
        ++frame;
    }

    void createLookup(size_t width, size_t height)
    {
        const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
        const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
        const float cx = cameraMatrixColor.at<double>(0, 2);
        const float cy = cameraMatrixColor.at<double>(1, 2);
        float *it;

        lookupY = cv::Mat(1, height, CV_32F);
        it = lookupY.ptr<float>();
        for(size_t r = 0; r < height; ++r, ++it)
        {
            *it = (r - cy) * fy;
        }

        lookupX = cv::Mat(1, width, CV_32F);
        it = lookupX.ptr<float>();
        for(size_t c = 0; c < width; ++c, ++it)
        {
            *it = (c - cx) * fx;
        }
    }

    static inline void setThreadName(const std::string &name)
    {
#if defined(__linux__)
        prctl(PR_SET_NAME, name.c_str());
#elif defined(__APPLE__)
        pthread_setname_np(name.c_str());
#endif
    }
};

void help(const std::string &path)
{
    std::cout << path << FG_BLUE " [options]" << std::endl
              << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR " equals to the kinect2_bridge topic base name" << std::endl
              << FG_GREEN "  mode" NO_COLOR ": " FG_YELLOW "'qhd'" NO_COLOR ", " FG_YELLOW "'hd'" NO_COLOR ", " FG_YELLOW "'sd'" NO_COLOR " or " FG_YELLOW "'ir'" << std::endl
              << FG_GREEN "  visualization" NO_COLOR ": " FG_YELLOW "'image'" NO_COLOR ", " FG_YELLOW "'cloud'" NO_COLOR " or " FG_YELLOW "'both'" << std::endl
              << FG_GREEN "  options" NO_COLOR ":" << std::endl
              << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
              << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
    ROSCONSOLE_AUTOINIT;
    if(!getenv("ROSCONSOLE_FORMAT"))
    {
        ros::console::g_formatter.tokens_.clear();
        ros::console::g_formatter.init("[${severity}] ${message}");
    }
#endif

    ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

    if(!ros::ok())
    {
        return 0;
    }

    std::string ns = K2_DEFAULT_NS;
    std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
    std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    bool useExact = true;
    bool useCompressed = false;
    bool showImage = false;
    Receiver::Mode mode = Receiver::IMAGE;

    for(size_t i = 1; i < (size_t)argc; ++i)
    {
        std::string param(argv[i]);

        if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
        {
            help(argv[0]);
            ros::shutdown();
            return 0;
        }
        else if(param == "qhd") // (960 x 540)
        {
            topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
            topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        }
        else if(param == "hd") // (1920 x 1080)
        {
            topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
            topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        }
        else if(param == "ir")
        {
            topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
            topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        }
        else if(param == "sd") // (512 x 424)
        {
            topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
            topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        }
        else if(param == "approx"){
            useExact = false;
        }
        else if(param == "compressed"){
            useCompressed = true;
        }
        else if(param == "image"){
            mode = Receiver::IMAGE;
        }
        else if(param == "cloud"){
            mode = Receiver::CLOUD;
        }
        else if(param == "both"){
            mode = Receiver::BOTH;
        }
        else if(param == "show"){
            showImage = true;
        }
        else{
            ns = param;
        }
    }

    topicColor = "/" + ns + topicColor;
    topicDepth = "/" + ns + topicDepth;
    OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
    OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

    Receiver receiver(topicColor, topicDepth, useExact, useCompressed, showImage);
    OUT_INFO("starting receiver...");

    ros::AsyncSpinner spinner(0);
    spinner.start();

//    ros::waitForShutdown();

    receiver.run(mode);

    spinner.stop();

    ros::shutdown();
    return 0;
}


