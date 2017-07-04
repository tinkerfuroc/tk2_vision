#include <tinker_depth_image_proc/filters/filters.h>
#include <cmath>
#include <ros/ros.h>

namespace tinker {
namespace vision {

using std::vector;

cv::Mat EntropyFilterMask(const cv::Mat & image, float threshold, int filter_size, int jump_step) {
    vector<float> entropy_table = BuildEntropyTable(filter_size);
    return EntropyFilterMask(image, threshold, filter_size, entropy_table, jump_step);
}

cv::Mat EntropyFilterMask(const cv::Mat & image, float threshold, int filter_size, const vector<float> & entropy_table, int jump_step) {
    cv::Mat mask(image.rows, image.cols, CV_8UC1);
    //build the entropy image
    cv::Mat entropy_mat(image.rows, image.cols, CV_32FC1);
    const cv::Mat & gray_scale_image = image;
    int gray_levels = filter_size * filter_size;
    int mov_start = filter_size / 2;
    int *gray_scale = new int[gray_levels];//buffer
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            if (i - mov_start < 0 || j - mov_start < 0
                || i - mov_start + filter_size >= image.rows ||
                j - mov_start + filter_size >= image.cols 
                || i % jump_step || j % jump_step)
                entropy_mat.at<float>(i, j) = 0;
            else {
                //build histogram by hand
                float entropy = 0;
                int x = i - mov_start;
                int y = j - mov_start;
                for (int k = 0; k < gray_levels; k++)
                    gray_scale[k] = 0;
                for (int movx = 0; movx < filter_size; movx++) {
                    for (int movy = 0; movy < filter_size; movy++) {
                        float gray = gray_scale_image.at<uchar>(movx + x, movy + y);
                        gray = gray / 256.0 * ((float) gray_levels);
                        gray_scale[int(gray)]++;
                    }
                }
                for (int k = 0; k < gray_levels; k++) {
                    float add_entropy = float(gray_scale[k]) * entropy_table[gray_scale[k]];
                    entropy += add_entropy;
                }
                entropy_mat.at<float>(i, j) = entropy / entropy_table[1] / float(gray_levels);
            }
        }
    }
    delete [] gray_scale;
    //filter the image
    for (int i = 0; i < entropy_mat.rows; i++)
        for (int j = 0; j < entropy_mat.cols; j++) {
            if (entropy_mat.at<float>(i, j) < threshold)
                mask.at<unsigned char>(i, j) = 0;
            else
                mask.at<unsigned char>(i, j) = 255;
        }
    return mask;
}

vector<float> BuildEntropyTable(int filter_size) {
    vector<float> entropy_table(filter_size*filter_size + 1);
    entropy_table[0] = 0;
    int gray_levels = filter_size * filter_size;
    for (int i = 1; i <= filter_size * filter_size; i++) {
        entropy_table[i] = log2((float) i / (float) gray_levels);
    }
    return entropy_table;
}

cv::Mat DepthFilterMask(const cv::Mat & loc_image, float min_depth, float max_depth) {
    cv::Mat mask(loc_image.rows, loc_image.cols, CV_8UC1);
    for (int i = 0; i < loc_image.rows; i++) {
        for (int j = 0; j < loc_image.cols; j++) {
            cv::Vec3s location = loc_image.at<cv::Vec3s>(i, j);
            float x = ((float)location[2]) / 1000.;
            if (min_depth < x && x < max_depth) {
                mask.at<unsigned char>(i, j) = 255;
            } else {
                mask.at<unsigned char>(i, j) = 0;
            }
        }
    }
    return mask;
}

void DilateImage(cv::Mat & image, int kernel_size) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
        cv::Point(kernel_size, kernel_size));
    cv::dilate(image, image, kernel);
}

void ErodeImage(cv::Mat & image, int kernel_size) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
        cv::Point(kernel_size, kernel_size));
    cv::erode(image, image, kernel);
}

static cv::Mat CannyDownSample(cv::Mat &img, int downsampleStep=10) { 
    cv::Mat d_img(img.rows/downsampleStep, img.cols/downsampleStep, CV_8UC1);
    cv::Mat o_img(img.rows/downsampleStep, img.cols/downsampleStep, CV_8UC1);   

    for (int y=0; y<d_img.rows; ++y)
        for (int x=0; x<d_img.cols; ++x) 
            o_img.at<uchar>(y,x) = d_img.at<uchar>(y,x) = 0;

    for (int y=0; y<img.rows; ++y)
        for (int x=0; x<img.cols; ++x) 
            if (img.at<uchar>(y,x))
                d_img.at<uchar>(y/downsampleStep, x/downsampleStep)=255;

    for (int y=1; y<d_img.rows-1; ++y)
        for (int x=1; x<d_img.cols-1; ++x)
            if (d_img.at<uchar>(y,x)==0) 
                o_img.at<uchar>(y,x)=0;
            else {
                if (d_img.at<uchar>(y-1,x) && 
                    d_img.at<uchar>(y+1,x) && 
                    d_img.at<uchar>(y,x-1) && 
                    d_img.at<uchar>(y,x+1)) 
                    o_img.at<uchar>(y,x)=0;
                else 
                    o_img.at<uchar>(y,x)=255;
            }
    return o_img;
}

static void bruteRemoveVerticals(cv::Mat &img, vector<cv::Vec4i> &lines) {
    for (int x=0; x<img.cols; ++x) {
        int cntPoint = 0, cntNull = 0;
        for (int y=0; y<img.rows; ++y) {
            if (img.at<uchar>(y,x)!=0) ++cntPoint; else ++cntNull;
            if (cntNull>=3) { cntNull=0; cntPoint=0;  }
            if (cntPoint > img.rows * 0.1) {
                while (y < img.rows && img.at<uchar>(y,x)!=0) { 
                    ++y; 
                    ++cntPoint;  
                }
                cv::Vec4i line;
                line[0] = line[2] = x;
                line[1] = y - cntPoint; 
                line[3] = y;
                lines.push_back(line);
            }
        }
    }

    for (int y=0; y<img.rows; ++y) {
        int cntPoint =0, cntNull = 0;
        for (int x=0; x<img.cols; ++x) {
            if (img.at<uchar>(y,x)!=0) 
                ++cntPoint; 
            else 
                ++cntNull;
            if (cntNull>=3) { 
                cntNull=0; 
                cntPoint = 0;
            }
            if (cntPoint > img.cols*0.1) {
                while (x<img.cols && img.at<uchar>(y,x)!=0) { 
                    ++x; 
                    ++cntPoint;
                }
                cv::Vec4i line;
                line[0]=x-cntPoint; line[2]=x;
                line[1] = line[3] = y;
                lines.push_back(line);
            }
        }
    }
}

cv::Mat LineFilterMask(const cv::Mat & image, double linewidth) {
    cv::Mat mask(image.rows, image.cols, CV_8UC1);
    for(int i = 0; i < mask.rows; i++) {
        for(int j = 0; j < mask.cols; j++) {
            mask.at<unsigned char>(i, j) = 255;
        }
    }
    int downsampleStep = 4;
    cv::Mat dst, cdst, d_dst;
    cv::Canny(image, dst, 50, 200, 3);
    d_dst = CannyDownSample(dst, downsampleStep);
    vector<cv::Vec4i> lines;
    cv::HoughLinesP(d_dst, lines, 1, CV_PI / 180, 30, (int)(d_dst.rows*0.3), 5);
    bruteRemoveVerticals(d_dst, lines);
    float width = downsampleStep * linewidth;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        l[0] *= downsampleStep; 
        l[1] *= downsampleStep; 
        l[2] *= downsampleStep; 
        l[3] *= downsampleStep;
        cv::line(mask, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), 0, width, 4);
    } 
    return mask;
}

}
}
