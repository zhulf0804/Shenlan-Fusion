#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Chessboard.h"
#include "CameraCalibration.h"

#include "apriltags/GridTargetAprilgrid.hpp"

int main(int argc, char** argv)
{
    cv::Size boardSize;
    float squareSize;
    std::string inputDir;
    std::string fileExtension;
    std::string cost_function_type;
    bool viewResults;
    bool verbose;
    bool use_apriltag;

    float apriltag_size = 0.088;
    float apriltag_interval = 0.3;

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(7), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(6), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(60.f), "Size of one square in mm")
        ("input,i", boost::program_options::value<std::string>(&inputDir)->default_value("calibrationdata"), "Input directory containing chessboard images")
        ("cost,c", boost::program_options::value<std::string>(&cost_function_type)->default_value("auto"), "cost function type of camera, auto & manual")
        ("extension,e", boost::program_options::value<std::string>(&fileExtension)->default_value(".png"), "File extension of images")
        ("view-results", boost::program_options::bool_switch(&viewResults)->default_value(true), "View results")
        ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(true), "Verbose output")
        ("apriltag-size", boost::program_options::value<float>(&apriltag_size)->default_value(0.088), "Size of apriltag in mm")
        ("apriltag-interval", boost::program_options::value<float>(&apriltag_interval)->default_value(0.3), "Interval of apriltag")
        ("apriltag,a", boost::program_options::bool_switch(&use_apriltag)->default_value(false), "use apriltag as target")
        ;

    boost::program_options::positional_options_description pdesc;
    pdesc.add("input", 1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pdesc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    if (!boost::filesystem::exists(inputDir) && !boost::filesystem::is_directory(inputDir))
    {
        std::cerr << "# ERROR: Cannot find input directory " << inputDir << "." << std::endl;
        return 1;
    }

    // look for images in input directory
    std::vector<std::string> imageFilenames;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if (!boost::filesystem::is_regular_file(itr->status()))
        {
            continue;
        }

        std::string filename = itr->path().filename().string();
        // check if file extension matches
        if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0)
        {
            continue;
        }

        imageFilenames.push_back(itr->path().string());

    }

    if (imageFilenames.empty())
    {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: # images: " << imageFilenames.size() << std::endl;
    }

    std::sort(imageFilenames.begin(), imageFilenames.end());

    cv::Mat image = cv::imread(imageFilenames.front(), -1);
    const cv::Size frameSize = image.size();

    CameraCalibration calibration(frameSize, boardSize, squareSize);
    calibration.setVerbose(verbose);

    std::vector<bool> chessboardFound(imageFilenames.size(), false);

    std::vector<bool> apriltagFound(imageFilenames.size(), false);

    if (use_apriltag) {
        GridCalibrationTargetAprilgrid aprilgrid=GridCalibrationTargetAprilgrid(6, 6,apriltag_size, apriltag_interval);
        
        // TODO: homework 4

        // 补全apriltag检测的代码，为标定提供 2D点和 3D点

        /////////////////////////////////////////////////////////////////
        for (size_t i = 0; i < imageFilenames.size(); i++){
            image = cv::imread(imageFilenames.at(i), 0);

            Eigen::MatrixXd outputPoints;
            std::vector<bool> outCornerObserved;
            aprilgrid.computeObservation(image, outputPoints, outCornerObserved);

            std::vector<cv::Point2f> corners;
            std::vector<cv::Point3f> tagpoints;
            for (int j = 0; j < outCornerObserved.size(); j++){
                if (outCornerObserved.at(j)){
                    corners.push_back(cv::Point2f(outputPoints(j, 0), outputPoints(j, 1)));
                    tagpoints.push_back(cv::Point3f(aprilgrid.point(j)[0], aprilgrid.point(j)[1], 0.0f));
                }
            }
            if (corners.size() > 16){
                calibration.addApriltagData(corners, tagpoints);
                apriltagFound[i] = true;
            }
        }

        //////////////////////////////////////////////////////////////

        cv::destroyAllWindows();
    } else {
        // use chessboard target 

        for (size_t i = 0; i < imageFilenames.size(); ++i)
        {
            image = cv::imread(imageFilenames.at(i), -1);

            Chessboard chessboard(boardSize, image);
            chessboard.findCorners();
            if (chessboard.cornersFound())
            {
                if (verbose)
                {
                    std::cerr << "# INFO: Detected chessboard in image " << i + 1 << ", " << imageFilenames.at(i) << std::endl;
                }

                calibration.addChessboardData(chessboard.getCorners());

                cv::Mat sketch;
                chessboard.getSketch().copyTo(sketch);

                // cv::imshow("Image", sketch);
                // cv::waitKey(50);
                
            }
            else if (verbose)
            {
                std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
            }
            chessboardFound.at(i) = chessboard.cornersFound();
        }
        
        // cv::destroyWindow("Image");
        

    }
    

    if (calibration.sampleCount() < 10)
    {
        std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: Calibrating..." << std::endl;
    }

    calibration.calibrate();

    if (viewResults)
    {
        std::vector<cv::Mat> cbImages;
        std::vector<std::string> cbImageFilenames;

        for (size_t i = 0; i < imageFilenames.size(); ++i)
        {
            if (!use_apriltag && !chessboardFound.at(i))
            {
                continue;
            }

            if (use_apriltag && !apriltagFound.at(i)){
                continue;
            }

            cbImages.push_back(cv::imread(imageFilenames.at(i), -1));
            cbImageFilenames.push_back(imageFilenames.at(i));
        }

        // visualize observed and reprojected points
        calibration.drawResults(cbImages);

        for (size_t i = 0; i < cbImages.size(); ++i)
        {
            cv::imshow("Image", cbImages.at(i));
            cv::waitKey();
        }
    }

    return 0;
}
