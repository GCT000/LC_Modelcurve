#include "curve.h"

using namespace lc_core;

void Curve::curveDetection(const cv::Mat &image)
{
    curve_lines_.clear();
    cv::Mat imgPre;
    // convert to gray
    cv::cvtColor(image, imgPre, cv::COLOR_BGR2GRAY);

    // Gaussian blur
    cv::GaussianBlur(imgPre, imgPre, cv::Size(5, 5), 0, 0);

    // Canny edge detection
    cv::Canny(imgPre, imgPre, 50, 150, 3);
    if (visualize_)
    {
        cv::imshow("canny", imgPre);
        cv::imwrite("canny.png", imgPre);
        cv::waitKey(0);
    }

    // dilate
    cv::Mat dilated;
    cv::dilate(imgPre, dilated, cv::Mat(), cv::Point(-1, -1), 2);

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    LOG(INFO) << "contours size: " << contours.size() << std::endl;
    if (visualize_)
    {
        cv::drawContours(dilated, contours, -1, cv::Scalar(222, 244, 255), 2);
        cv::imshow("contours", dilated);
        cv::waitKey(0);
    }

    // whether the contour is a curve
    for (int i = 0; i < contours.size(); i++)
    {
        LOG(INFO) << "contours[" << i + 1 << "].size(): " << contours[i].size() << std::endl;
        if (contours[i].size() > 100)
        {
            curve_lines_.push_back(contours[i]);
        }
    }

    LOG(WARNING) << "detect " << curve_lines_.size() << " curves." << std::endl;
}

[[maybe_unused]] void Curve::featuresDetection(const cv::Mat &image)
{
    auto orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb->detectAndCompute(image, cv::Mat(), keypoints, descriptors);

    cv::Mat img_keypoints;
    cv::drawKeypoints(image, keypoints, img_keypoints);
    if (visualize_)
    {
        cv::imshow("keypoints", img_keypoints);
        cv::waitKey(0);
    }
}