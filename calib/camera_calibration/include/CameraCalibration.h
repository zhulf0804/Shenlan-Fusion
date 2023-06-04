#pragma once

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include "PinholeCamera.h"

class CameraCalibration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraCalibration();

    CameraCalibration(const cv::Size& imageSize,
                      const cv::Size& boardSize,
                      float squareSize);

    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& corners);

    void addApriltagData(const std::vector<cv::Point2f>& corners,
                        const std::vector<cv::Point3f>& tagpoints);

    bool calibrate(void);

    int sampleCount(void) const;
    std::vector<std::vector<cv::Point2f> >& imagePoints(void);
    const std::vector<std::vector<cv::Point2f> >& imagePoints(void) const;
    std::vector<std::vector<cv::Point3f> >& scenePoints(void);
    const std::vector<std::vector<cv::Point3f> >& scenePoints(void) const;
    PinholeCameraPtr& camera(void);

    Eigen::Matrix2d& measurementCovariance(void);
    const Eigen::Matrix2d& measurementCovariance(void) const;

    cv::Mat& cameraPoses(void);
    const cv::Mat& cameraPoses(void) const;

    void drawResults(std::vector<cv::Mat>& images) const;

    void setVerbose(bool verbose);

private:
    bool calibrateHelper(PinholeCameraPtr& camera,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    void optimize(PinholeCameraPtr& camera,
                  std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    cv::Size m_boardSize;
    float m_squareSize;

    PinholeCameraPtr m_camera;
    cv::Mat m_cameraPoses;

    std::vector<std::vector<cv::Point2f> > m_imagePoints;
    std::vector<std::vector<cv::Point3f> > m_scenePoints;

    Eigen::Matrix2d m_measurementCovariance;

    bool m_verbose;
};
