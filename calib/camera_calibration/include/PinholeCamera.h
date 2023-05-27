#pragma once 

#include <opencv2/core/core.hpp>
#include <string>

#include "ceres/rotation.h"

#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

class PinholeCamera
{

public:
    class Parameters
    {
    public:
        Parameters();
        Parameters(int w, int h,
                   double k1 = 0, double k2 = 0, double p1 = 0, double p2 = 0, 
                   double fx = 0, double fy = 0, double cx = 0, double cy = 0);

        double& k1(){ return m_k1; }
        double& k2(){ return m_k2; }
        double& p1(){ return m_p1; }
        double& p2(){ return m_p2; }
        double& fx(){ return m_fx; }
        double& fy(){ return m_fy; }
        double& cx(){ return m_cx; }
        double& cy(){ return m_cy; }

        double k1(void) const { return m_k1; }
        double k2(void) const { return m_k2; }
        double p1(void) const { return m_p1; }
        double p2(void) const { return m_p2; }
        double fx(void) const { return m_fx; }
        double fy(void) const { return m_fy; }
        double cx(void) const { return m_cx; }
        double cy(void) const { return m_cy; }

        int& imageHeight() { return m_imageHeight; }
        int& imageWidth() { return m_imageWidth;}

        int imageHeight() const { return m_imageHeight; }
        int imageWidth() const { return m_imageWidth; }

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_fx;
        double m_fy;
        double m_cx;
        double m_cy;

        int m_imageWidth;
        int m_imageHeight;
    };

    PinholeCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    PinholeCamera(int imageWidth, int imageHeight,
                  double k1, double k2, double p1, double p2,
                  double fx, double fy, double cx, double cy);
    /**
    * \brief Constructor from the projection model parameters
    */
    PinholeCamera(const Parameters& params);

    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsics(const cv::Size& boardSize,
                            const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints);

    void estimateExtrinsics(const std::vector<cv::Point3f>& objectPoints,
                           const std::vector<cv::Point2f>& imagePoints,
                           cv::Mat& rvec, cv::Mat& tvec) const;

    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    //%output p

    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;

    int parameterCount(void) const;

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    void readParameters(const std::vector<double>& parameterVec);
    void writeParameters(std::vector<double>& parameterVec) const;

    std::string parametersToString(void) const;

        /**
     * \brief Calculates the reprojection distance between points
     *
     * \param P1 first 3D point coordinates
     * \param P2 second 3D point coordinates
     * \return euclidean distance in the plane
     */
    double reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const;

    double reprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                             const std::vector< std::vector<cv::Point2f> >& imagePoints,
                             const std::vector<cv::Mat>& rvecs,
                             const std::vector<cv::Mat>& tvecs) const;

    double reprojectionError(const Eigen::Vector3d& P,
                             const Eigen::Quaterniond& camera_q,
                             const Eigen::Vector3d& camera_t,
                             const Eigen::Vector2d& observed_p) const;

    void projectPoints(const std::vector<cv::Point3f>& objectPoints,
                       const cv::Mat& rvec,
                       const cv::Mat& tvec,
                       std::vector<cv::Point2f>& imagePoints) const;

private:
    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;

    cv::Mat m_mask;
};

typedef boost::shared_ptr<PinholeCamera> PinholeCameraPtr;
typedef boost::shared_ptr<const PinholeCamera> PinholeCameraConstPtr;

