// Foreground Detector.
// Du 16/01/15.
//
#include <tinker_object_recognition/arm_target_finder/ForegroundDetector.h>
#include <cassert>
#include <cmath>
#include <cfloat>

using std::vector;

namespace tinker {
namespace vision {
void ForegroundDetector::OpenImage(cv::Mat &mat) {
    DilateImage(mat, 3);
    ErodeImage(mat, 6);
    DilateImage(mat, 8);
}

void ForegroundDetector::FilterByEntropy(const cv::Mat &source_mat,
                                         cv::Mat &desk_mat) {
    cv::Mat mat_grey(source_mat.rows, source_mat.cols, CV_8UC1);
    cv::cvtColor(source_mat, mat_grey, CV_BGR2GRAY);
    desk_mat = EntropyFilterMask(mat_grey, entropy_threshold_, filter_size_, 4);
}

cv::Mat ForegroundDetector::BuildMask(const cv::Mat &mat) {
    // get threshold of desk_mat
    cv::Mat mat_threshold(mat);
    cv::threshold(mat_threshold, mat_threshold, 10, 255, cv::THRESH_BINARY);
    // find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mat_threshold, contours, hierarchy, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);
    // find biggest contour
    double largest_area = 0.0;
    int largest_contour_index = 0;
    for (int i = 0; i < contours.size(); i++) {
        double a = cv::contourArea(contours[i], false);
        if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;
        }
    }
    // find minor biggest contour
    std::vector<int> bigcontours_index;
    for (int i = 0; i < contours.size(); i++) {
        double a = cv::contourArea(contours[i], false);
        if (a > largest_area / 2) {
            bigcontours_index.push_back(i);
        }
    }
    // draw big contours in contour_mat
    cv::Mat contour_mat(mat.rows, mat.cols, CV_8UC1, cv::Scalar::all(0));
    cv::Scalar whitecolor(255, 255, 255);
    for (int i = 0; i < bigcontours_index.size(); i++) {
        cv::drawContours(contour_mat, contours, bigcontours_index[i],
                         whitecolor, CV_FILLED, 8, hierarchy);
    }
    return contour_mat;
}

void ForegroundDetector::DivideMaskByY(cv::Mat &mask_mat) {
    // split by y
    int *integral_on_x = new int[mask_mat.cols];
    for (int j = 0; j < mask_mat.cols; j++) {
        integral_on_x[j] = 0;
        for (int i = 0; i < mask_mat.rows; i++) {
            if (mask_mat.at<uchar>(i, j) == 255) {
                integral_on_x[j]++;
            }
        }
    }
    int max_intergral_on_x = 0;
    for (int j = 0; j < mask_mat.cols; j++) {
        if (max_intergral_on_x < integral_on_x[j]) {
            max_intergral_on_x = integral_on_x[j];
        }
    }
    for (int i = 0; i < mask_mat.rows; i++) {
        for (int j = 0; j < mask_mat.cols; j++) {
            if (integral_on_x[j] < max_intergral_on_x / 4) {
                mask_mat.at<uchar>(i, j) = 0;
            }
        }
    }
    delete [] integral_on_x;
}

void ForegroundDetector::BuildImageByMask(const cv::Mat &source_mat,
                                          cv::Mat &desk_mat, cv::Mat mask_mat) {
    assert(desk_mat.rows == source_mat.rows);
    assert(desk_mat.cols == source_mat.cols);
    // finally get desk_mat
    for (int i = 0; i < source_mat.rows; i++) {
        for (int j = 0; j < source_mat.cols; j++) {
            if (mask_mat.at<uchar>(i, j) == 255) {
                desk_mat.at<cv::Vec3b>(i, j) = source_mat.at<cv::Vec3b>(i, j);
            } else {
                desk_mat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }
}

void ForegroundDetector::Filter(const cv::Mat &source_mat, cv::Mat &desk_mat) {
    // filter by entropy
    FilterByEntropy(source_mat, desk_mat);
    // dilate, erode, and dilate again
    OpenImage(desk_mat);
    // build mask
    cv::Mat mask_mat = BuildMask(desk_mat);
    DilateImage(mask_mat, 3);
    // divide mask by y
    DivideMaskByY(mask_mat);
    // Build foreground
    BuildImageByMask(source_mat, desk_mat, mask_mat);
}

vector<ForegroundImage> ForegroundDetector::CutAllForegroundOut(const cv::Mat &source_mat) {
    vector<ForegroundImage> results;
    // copy the source
    cv::Mat res_mat;
    source_mat.copyTo(res_mat);
    // filter by entropy
    FilterByEntropy(source_mat, res_mat);
    // dilate, erode, and dilate again
    OpenImage(res_mat);
    // build mask
    cv::Mat mask_mat = BuildMask(res_mat);
    DilateImage(mask_mat, 3);
    // divide mask by y
    DivideMaskByY(mask_mat);
    // find contour again(this time divided)
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_mat, contours, hierarchy, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++) {
        cv::Rect contour_bound = cv::boundingRect(contours[i]);
        double ar = (double)contour_bound.width / (double)contour_bound.height;
        if(ar > 1/max_aspect_ratio_tolerance_ 
           && ar < max_aspect_ratio_tolerance_) {
            double a = cv::contourArea(contours[i], false);
            if (a > min_squaresize_) {
                ForegroundImage foreground;
                foreground.foreground = source_mat(contour_bound);
                foreground.bound = contour_bound;
                results.push_back(foreground);
            }
        }
    }
    return results;
}

int ForegroundDetector::CutForegroundOut(const cv::Mat &source_mat,
                                         cv::Mat &desk_mat, cv::Rect &bound) {
    vector<ForegroundImage> foregrounds = CutAllForegroundOut(source_mat);
    if (foregrounds.empty()) return ForegroundDetector::NOT_DETECTED;
    int result_id = -1;
    float min_center_distance = FLT_MAX;
    for (int i = 0; i < foregrounds.size(); i++) {
        float foreground_center =
            foregrounds[i].bound.x + foregrounds[i].bound.width / 2;
        float center_distance = fabs(foreground_center - source_mat.cols / 2);
        if (center_distance < min_center_distance) {
            result_id = i;
            min_center_distance = center_distance;
        }
    }
    desk_mat = foregrounds[result_id].foreground;
    bound = foregrounds[result_id].bound;
    return ForegroundDetector::DETECTED;
}

int ForegroundDetector::CutForegroundOut(const cv::Mat &source_mat, cv::Mat &desk_mat) {
    cv::Rect bound;
    return CutForegroundOut(source_mat, desk_mat, bound);
}


}
}
