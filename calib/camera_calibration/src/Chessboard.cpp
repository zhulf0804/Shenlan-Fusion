#include "Chessboard.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/calib3d/calib3d_c.h>


Chessboard::Chessboard(cv::Size boardSize, cv::Mat& image)
 : mBoardSize(boardSize)
 , mCornersFound(false)
{
    if (image.channels() == 1)
    {
        cv::cvtColor(image, mSketch, CV_GRAY2BGR);
        image.copyTo(mImage);
    }
    else
    {
        image.copyTo(mSketch);
        cv::cvtColor(image, mImage, CV_BGR2GRAY);
    }
}

void
Chessboard::findCorners()
{
    mCornersFound = findChessboardCorners(mImage, mBoardSize, mCorners,
                                          CV_CALIB_CB_ADAPTIVE_THRESH +
                                          CV_CALIB_CB_NORMALIZE_IMAGE +
                                          CV_CALIB_CB_FILTER_QUADS +
                                          CV_CALIB_CB_FAST_CHECK);

    if (mCornersFound)
    {
        // draw chessboard corners
        cv::drawChessboardCorners(mSketch, mBoardSize, mCorners, mCornersFound);
    }
}

const std::vector<cv::Point2f>&
Chessboard::getCorners(void) const
{
    return mCorners;
}

bool
Chessboard::cornersFound(void) const
{
    return mCornersFound;
}

const cv::Mat&
Chessboard::getImage(void) const
{
    return mImage;
}

const cv::Mat&
Chessboard::getSketch(void) const
{
    return mSketch;
}

bool
Chessboard::findChessboardCorners(const cv::Mat& image,
                                  const cv::Size& patternSize,
                                  std::vector<cv::Point2f>& corners,
                                  int flags)
{
    if(cv::findChessboardCorners(image, patternSize, corners, flags)){
        cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        return true;
    }

    return false;
}
