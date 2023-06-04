#include "CameraCalibration.h"

#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "PinholeCamera.h"
#include "EigenQuaternionParameterization.h"
#include "EigenUtils.h"
#include "CostFunctionFactory.h"
#include "Transform.h"

#include "ceres/ceres.h"

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

CameraCalibration::CameraCalibration()
 : m_boardSize(cv::Size(0,0))
 , m_squareSize(0.0f)
 , m_verbose(false)
{

}

CameraCalibration::CameraCalibration(const cv::Size& imageSize,
                                     const cv::Size& boardSize,
                                     float squareSize)
 : m_boardSize(boardSize)
 , m_squareSize(squareSize)
 , m_verbose(false)
{   
    PinholeCameraPtr camera(new PinholeCamera);
    PinholeCamera::Parameters params = camera->getParameters();
    params.imageWidth() = imageSize.width;
    params.imageHeight() = imageSize.height;
    camera->setParameters(params);

    m_camera = camera;
}

void
CameraCalibration::clear(void)
{
    m_imagePoints.clear();
    m_scenePoints.clear();
}

void
CameraCalibration::addChessboardData(const std::vector<cv::Point2f>& corners)
{
    m_imagePoints.push_back(corners);

    std::vector<cv::Point3f> scenePointsInView;

    // TODO: homework1

    // 根据棋盘格标定板的属性，生成标定板上边3D点在三维空间中的位置

    ////////////////////////////////////////////////////////////////////
    for (int i = 0; i < m_boardSize.height; i++){
        for (int j = 0; j < m_boardSize.width; j++){
            float x = i * m_squareSize;
            float y = j * m_squareSize;
            cv::Point3f point3d(x, y, 0.0f);
            scenePointsInView.push_back(point3d);
        }
    }
    ////////////////////////////////////////////////////////////////////

    m_scenePoints.push_back(scenePointsInView);
}

void 
CameraCalibration::addApriltagData(const std::vector<cv::Point2f>& corners,
                                   const std::vector<cv::Point3f>& tagpoints)
{
    m_imagePoints.push_back(corners);
    m_scenePoints.push_back(tagpoints);
}

bool
CameraCalibration::calibrate(void)
{
    int imageCount = m_imagePoints.size();

    // compute intrinsic camera parameters and extrinsic parameters for each of the views
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    bool ret = calibrateHelper(m_camera, rvecs, tvecs);

    m_cameraPoses = cv::Mat(imageCount, 6, CV_64F);
    for (int i = 0; i < imageCount; ++i)
    {
        m_cameraPoses.at<double>(i,0) = rvecs.at(i).at<double>(0);
        m_cameraPoses.at<double>(i,1) = rvecs.at(i).at<double>(1);
        m_cameraPoses.at<double>(i,2) = rvecs.at(i).at<double>(2);
        m_cameraPoses.at<double>(i,3) = tvecs.at(i).at<double>(0);
        m_cameraPoses.at<double>(i,4) = tvecs.at(i).at<double>(1);
        m_cameraPoses.at<double>(i,5) = tvecs.at(i).at<double>(2);
    }

    // Compute measurement covariance.
    std::vector<std::vector<cv::Point2f> > errVec(m_imagePoints.size());
    Eigen::Vector2d errSum = Eigen::Vector2d::Zero();
    size_t errCount = 0;
    for (size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        std::vector<cv::Point2f> estImagePoints;
        m_camera->projectPoints(m_scenePoints.at(i), rvecs.at(i), tvecs.at(i),
                                estImagePoints);

        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            cv::Point2f pObs = m_imagePoints.at(i).at(j);
            cv::Point2f pEst = estImagePoints.at(j);

            cv::Point2f err = pObs - pEst;

            errVec.at(i).push_back(err);

            errSum += Eigen::Vector2d(err.x, err.y);
        }

        errCount += m_imagePoints.at(i).size();
    }

    Eigen::Vector2d errMean = errSum / static_cast<double>(errCount);

    Eigen::Matrix2d measurementCovariance = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < errVec.size(); ++i)
    {
        for (size_t j = 0; j < errVec.at(i).size(); ++j)
        {
            cv::Point2f err = errVec.at(i).at(j);
            double d0 = err.x - errMean(0);
            double d1 = err.y - errMean(1);

            measurementCovariance(0,0) += d0 * d0;
            measurementCovariance(0,1) += d0 * d1;
            measurementCovariance(1,1) += d1 * d1;
        }
    }
    measurementCovariance /= static_cast<double>(errCount);
    measurementCovariance(1,0) = measurementCovariance(0,1);

    m_measurementCovariance = measurementCovariance;

    return ret;
}

int
CameraCalibration::sampleCount(void) const
{
    return m_imagePoints.size();
}

std::vector<std::vector<cv::Point2f> >&
CameraCalibration::imagePoints(void)
{
    return m_imagePoints;
}

const std::vector<std::vector<cv::Point2f> >&
CameraCalibration::imagePoints(void) const
{
    return m_imagePoints;
}

std::vector<std::vector<cv::Point3f> >&
CameraCalibration::scenePoints(void)
{
    return m_scenePoints;
}

const std::vector<std::vector<cv::Point3f> >&
CameraCalibration::scenePoints(void) const
{
    return m_scenePoints;
}

PinholeCameraPtr&
CameraCalibration::camera(void)
{
    return m_camera;
}

Eigen::Matrix2d&
CameraCalibration::measurementCovariance(void)
{
    return m_measurementCovariance;
}

const Eigen::Matrix2d&
CameraCalibration::measurementCovariance(void) const
{
    return m_measurementCovariance;
}

cv::Mat&
CameraCalibration::cameraPoses(void)
{
    return m_cameraPoses;
}

const cv::Mat&
CameraCalibration::cameraPoses(void) const
{
    return m_cameraPoses;
}

void
CameraCalibration::drawResults(std::vector<cv::Mat>& images) const
{
    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat& image = images.at(i);
        if (image.channels() == 1)
        {
            cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
        }

        float errorSum = 0.0f;
        float errorMax = std::numeric_limits<float>::min();

        // TODO homework 3

        // 在图像中画出检测点和重投影点，并计算最大的重投影误差和平均的重投影误差

        ////////////////////////////////////////////////////////////////////////
        // cv::Mat rvec(3, 1, CV_64F);
        // rvec.at<double>(0) = m_cameraPoses.at<double>(i, 0);
        // rvec.at<double>(1) = m_cameraPoses.at<double>(i, 1);
        // rvec.at<double>(2) = m_cameraPoses.at<double>(i, 2);
        // cv::Mat tvec(3, 1, CV_64F);
        // tvec.at<double>(0) = m_cameraPoses.at<double>(i, 3);
        // tvec.at<double>(1) = m_cameraPoses.at<double>(i, 4);
        // tvec.at<double>(2) = m_cameraPoses.at<double>(i, 5);
        // std::vector<cv::Point2f> estImagePoints;
        // m_camera->projectPoints(m_scenePoints.at(i), rvec, tvec, estImagePoints);

        std::vector<cv::Point2f> estImagePoints;
        m_camera->projectPoints(m_scenePoints.at(i), cameraPoses().rowRange(i, i+1).colRange(0, 3), 
                                cameraPoses().rowRange(i, i+1).colRange(3, 6), estImagePoints);
        for (size_t j = 0; j < estImagePoints.size(); ++j){
            cv::circle(image, estImagePoints.at(j), 5, green, 2, CV_AA);
            cv::circle(image, m_imagePoints.at(i).at(j), 5, red, 2, CV_AA);
            float error = cv::norm(estImagePoints.at(j) - m_imagePoints.at(i).at(j));
            errorSum += error;
            errorMax = fmax(errorMax, error);
        }


        /////////////////////////////////////////////////////////////

        std::ostringstream oss;
        oss << "Reprojection error: avg = " << errorSum / m_imagePoints.at(i).size()
            << "   max = " << errorMax;

        cv::putText(image, oss.str(), cv::Point(10, image.rows - 10),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);
    }
}

void
CameraCalibration::setVerbose(bool verbose)
{
    m_verbose = verbose;
}

bool
CameraCalibration::calibrateHelper(PinholeCameraPtr& camera,
                                   std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const
{
    rvecs.assign(m_scenePoints.size(), cv::Mat());
    tvecs.assign(m_scenePoints.size(), cv::Mat());

    // STEP 1: Estimate intrinsics
    camera->estimateIntrinsics(m_boardSize, m_scenePoints, m_imagePoints);

    // STEP 2: Estimate extrinsics
    for (size_t i = 0; i < m_scenePoints.size(); ++i)
    {
        camera->estimateExtrinsics(m_scenePoints.at(i), m_imagePoints.at(i), rvecs.at(i), tvecs.at(i));
    }

    if (m_verbose)
    {
        std::cout << "# INFO: " << "Initial reprojection error: "
                  << std::fixed << std::setprecision(3)
                  << camera->reprojectionError(m_scenePoints, m_imagePoints, rvecs, tvecs)
                  << " pixels" << std::endl;
    }

    // STEP 3: optimization using ceres
    optimize(camera, rvecs, tvecs);

    if (m_verbose)
    {
        double err = camera->reprojectionError(m_scenePoints, m_imagePoints, rvecs, tvecs);
        std::cout  << "# INFO: Final reprojection error: "
                  << err << " pixels" << std::endl;
        std::cout  << "# INFO: "
                  << camera->parametersToString() << std::endl;
    }

    return true;
}

void
CameraCalibration::optimize(PinholeCameraPtr& camera,
                            std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const
{
    // Use ceres to do optimization
    ceres::Problem problem;

    std::vector<Transform, Eigen::aligned_allocator<Transform> > transformVec(rvecs.size());
    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        Eigen::Vector3d rvec;
        cv::cv2eigen(rvecs.at(i), rvec);

        transformVec.at(i).rotation() = Eigen::AngleAxisd(rvec.norm(), rvec.normalized());
        transformVec.at(i).translation() << tvecs[i].at<double>(0),
                                            tvecs[i].at<double>(1),
                                            tvecs[i].at<double>(2);
    }

    std::vector<double> intrinsicCameraParams;
    m_camera->writeParameters(intrinsicCameraParams);

    // create residuals for each observation
    for (size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            const cv::Point3f& spt = m_scenePoints.at(i).at(j);
            const cv::Point2f& ipt = m_imagePoints.at(i).at(j);

            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(camera,
                                                                      Eigen::Vector3d(spt.x, spt.y, spt.z),
                                                                      Eigen::Vector2d(ipt.x, ipt.y));

            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);
            problem.AddResidualBlock(costFunction, lossFunction,
                                     intrinsicCameraParams.data(),
                                     transformVec.at(i).rotationData(),
                                     transformVec.at(i).translationData());
        }

        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization(transformVec.at(i).rotationData(),
                                    quaternionParameterization);
    }

    std::cout << "begin ceres" << std::endl;
    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.num_threads = 4;

    if (m_verbose)
    {
        options.minimizer_progress_to_stdout = true;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    if (m_verbose)
    {
        std::cout << summary.FullReport() << std::endl;
    }

    camera->readParameters(intrinsicCameraParams);

    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        Eigen::AngleAxisd aa(transformVec.at(i).rotation());

        Eigen::Vector3d rvec = aa.angle() * aa.axis();
        cv::eigen2cv(rvec, rvecs.at(i));

        cv::Mat& tvec = tvecs.at(i);
        tvec.at<double>(0) = transformVec.at(i).translation()(0);
        tvec.at<double>(1) = transformVec.at(i).translation()(1);
        tvec.at<double>(2) = transformVec.at(i).translation()(2);
    }
}
