#pragma once

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

class Chessboard
{
public:
    Chessboard(cv::Size boardSize, cv::Mat& image);

    void findCorners();
    const std::vector<cv::Point2f>& getCorners(void) const;
    bool cornersFound(void) const;

    const cv::Mat& getImage(void) const;
    const cv::Mat& getSketch(void) const;

private:
    bool findChessboardCorners(const cv::Mat& image,
                               const cv::Size& patternSize,
                               std::vector<cv::Point2f>& corners,
                               int flags);

    cv::Mat mImage;
    cv::Mat mSketch;
    std::vector<cv::Point2f> mCorners;
    cv::Size mBoardSize;
    bool mCornersFound;
};

