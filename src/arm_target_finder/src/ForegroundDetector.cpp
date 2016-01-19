// Foreground Detector.
// Du 16/01/15.
//
#include <arm_target_finder/ForegroundDetector.h>
#include <cassert>
#include <cmath>

void FUCK(int x){ROS_INFO("fuck %d", x);}
void FUCK(float x){ROS_INFO("fuck %f", x);}
void FUCK(char *x){ROS_INFO("fuck %s", x);}

namespace tinker
{
    namespace vision
    {
        ForegroundDetector::ForegroundDetector(int filter_size, double entropy_threshold, double similarity_threshold)
            :filter_size_(filter_size), entropy_threshold_(entropy_threshold), similarity_threshold_(similarity_threshold)
        {
            entropy_table_ = new double[filter_size*filter_size + 1];
            BuildEntropyTable_();
        }
        
        ForegroundDetector::~ForegroundDetector()
        {
            delete [] entropy_table_;
        }
        
        void ForegroundDetector::BuildEntropyTable_()
        {
            entropy_table_[0] = 0;
            int gray_levels = filter_size_ * filter_size_;
            for (int i = 1; i <= filter_size_ * filter_size_; i++)
            {
                entropy_table_[i] = log2((double) i / (double) gray_levels);
            }
        }
        
        
        void ForegroundDetector::DilateImage(cv::Mat & mat, int kernel_size)
        {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
                cv::Point(kernel_size, kernel_size));
            cv::dilate(mat, mat, kernel);
        }
        
        void ForegroundDetector::ErodeImage(cv::Mat & mat, int kernel_size)
        {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
                cv::Point(kernel_size, kernel_size));
            cv::erode(mat, mat, kernel);
        }
        
        void ForegroundDetector::OpenImage(cv::Mat & mat)
        {
            DilateImage(mat, 3);
            ErodeImage(mat, 6);
            DilateImage(mat, 8);
        }
        
        void ForegroundDetector::FilterByEntropy(const cv::Mat & source_mat, cv::Mat & desk_mat)
        {
            //build the entropy image
            cv::Mat entropy_mat(source_mat.rows, source_mat.cols, CV_64FC1);
            cv::Mat gray_scale_image;
            cv::cvtColor(source_mat, gray_scale_image, CV_BGR2GRAY);
            int gray_levels = filter_size_ * filter_size_;
            int mov_start = filter_size_ / 2;
            int *gray_scale = new int[gray_levels];//buffer
            for (int i = 0; i < source_mat.rows; i++)
            {
                for (int j = 0; j < source_mat.cols; j++)
                {
                    if (i - mov_start < 0 || j - mov_start < 0
                        || i - mov_start + filter_size_ >= source_mat.rows ||
                        j - mov_start + filter_size_ >= source_mat.cols 
                        || i%4 || j%4)
                    {
                        entropy_mat.at<double>(i, j) = 0;
                    }
                    else
                    {
                        //build histogram by hand
                        double entropy = 0;
                        int x = i - mov_start;
                        int y = j - mov_start;
                        for (int k = 0; k < gray_levels; k++)
                        {
                            gray_scale[k] = 0;
                        }
                        for (int movx = 0; movx < filter_size_; movx++)
                        {
                            for (int movy = 0; movy < filter_size_; movy++)
                            {
                                double gray = gray_scale_image.at<uchar>(movx + x, movy + y);
                                gray = gray / 256.0 * ((double) gray_levels);
                                gray_scale[int(gray)]++;
                            }
                        }
                        for (int k = 0; k < gray_levels; k++)
                        {
                            double add_entropy = double(gray_scale[k]) * entropy_table_[gray_scale[k]];
                            entropy += add_entropy;
                        }
                        entropy_mat.at<double>(i, j) = entropy / entropy_table_[1] / double(gray_levels);
                    }
                }
            }
            delete [] gray_scale;
            //filter the image
            for (int i = 0; i < entropy_mat.rows; i++)
            {
                for (int j = 0; j < entropy_mat.cols; j++)
                {
                    if (entropy_mat.at<double>(i, j) < entropy_threshold_)
                    {
                        desk_mat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
                    }
                }
            }
        }
        
        cv::Mat ForegroundDetector::BuildMask(const cv::Mat & mat)
        {
            //get threshold of desk_mat
            cv::Mat mat_threshold(mat.rows, mat.cols, CV_8UC1); 
            cv::cvtColor(mat, mat_threshold, CV_BGR2GRAY);
            cv::threshold(mat_threshold, mat_threshold, 10, 255, cv::THRESH_BINARY);
            //find contours
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mat_threshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            //find biggest contour
            double largest_area = 0.0;
            int largest_contour_index = 0;
            for(int i = 0; i < contours.size(); i++ )
            {
                double a = cv::contourArea(contours[i], false);
                if(a > largest_area)
                {
                    largest_area=a;
                    largest_contour_index=i;
                }
            }
            //find minor biggest contour
            std::vector<int> bigcontours_index;
            for(int i = 0; i < contours.size(); i++ )
            {
                double a = cv::contourArea(contours[i], false);
                if(a > largest_area / 2)
                {
                    bigcontours_index.push_back(i);
                }
            }
            //draw big contours in contour_mat
            cv::Mat contour_mat(mat.rows, mat.cols, CV_8UC1, cv::Scalar::all(0));
            cv::Scalar whitecolor(255,255,255);
            for(int i = 0; i < bigcontours_index.size(); i++ )
            {
                cv::drawContours(contour_mat, contours, bigcontours_index[i], whitecolor, CV_FILLED, 8, hierarchy);
            }
            return contour_mat;
        }
        
        void ForegroundDetector::DivideMaskByY(cv::Mat & mask_mat)
        {
            //split by y
            int *integral_on_x = new int[mask_mat.cols];
            for (int j = 0; j < mask_mat.cols; j++)
            {
                integral_on_x[j] = 0;
                for (int i = 0; i < mask_mat.rows; i++)
                {
                    if (mask_mat.at<uchar>(i, j) == 255)
                    {
                        integral_on_x[j]++;
                    }
                }
            }
            int max_intergral_on_x = 0;
            for (int j = 0; j < mask_mat.cols; j++)
            {
                if (max_intergral_on_x < integral_on_x[j])
                {
                    max_intergral_on_x = integral_on_x[j];
                }
            }
            for (int i = 0; i < mask_mat.rows; i++)
            {
                for (int j = 0; j < mask_mat.cols; j++)
                {
                    if (integral_on_x[j] < max_intergral_on_x / 4)
                    {
                        mask_mat.at<uchar>(i, j) = 0;
                    }
                }
            }
            delete integral_on_x;
        }
        
        void ForegroundDetector::BuildImageByMask(const cv::Mat & source_mat, cv::Mat & desk_mat, cv::Mat mask_mat)
        {
            assert(desk_mat.rows == source_mat.rows);
            assert(desk_mat.cols == source_mat.cols);
            //finally get desk_mat
            for (int i = 0; i < source_mat.rows; i++)
            {
                for (int j = 0; j < source_mat.cols; j++)
                {
                    if (mask_mat.at<uchar>(i, j) == 255)
                    {
                        desk_mat.at<cv::Vec3b>(i, j) = source_mat.at<cv::Vec3b>(i, j);
                    }
                    else
                    {
                        desk_mat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
                    }
                }
            }
        }
        
        void ForegroundDetector::Filter(const cv::Mat & source_mat, cv::Mat & desk_mat)
        {
            //filter by entropy
            FilterByEntropy(source_mat, desk_mat);
            //dilate, erode, and dilate again
            OpenImage(desk_mat);
            //build mask
            cv::Mat mask_mat = BuildMask(desk_mat);
            DilateImage(mask_mat, 3);
            //divide mask by y
            DivideMaskByY(mask_mat);
            //Build foreground
            BuildImageByMask(source_mat, desk_mat, mask_mat);
        }
        
        cv::Vec3b ForegroundDetector::bar_(cv::Mat m1, cv::Mat m2)
        {
            assert(m1.rows == m2.rows);
            assert(m1.cols == m2.cols);
            assert(m1.depth() == m2.depth());
            assert(m1.channels() == m2.channels());
            assert(m1.depth() == CV_8U);
            assert(m1.channels() == 3);
            double r = 0;
            const static int bar = 128;
            int sigmax = 0;
            int sigmay = 0;
            int sigmaz = 0;
            for (int i=0; i<m1.rows; i++)
            {
                for (int j=0; j<m1.cols; j++)
                {
                    cv::Vec3b m1ij = m1.at<cv::Vec3b>(i, j);
                    cv::Vec3b m2ij = m2.at<cv::Vec3b>(i, j);
                    sigmax += m1ij[0] + m2ij[0];
                    sigmay += m1ij[1] + m2ij[1];
                    sigmaz += m1ij[2] + m2ij[2];
                }
            }
            sigmax /= (m1.rows * m1.cols * 2);
            sigmay /= (m1.rows * m1.cols * 2);
            sigmaz /= (m1.rows * m1.cols * 2);
            return cv::Vec3b(sigmax, sigmay, sigmaz);
        }
        
        double ForegroundDetector::dot_(cv::Mat m1, cv::Mat m2, cv::Vec3b offset)
        {
            assert(m1.rows == m2.rows);
            assert(m1.cols == m2.cols);
            assert(m1.depth() == m2.depth());
            assert(m1.channels() == m2.channels());
            assert(m1.depth() == CV_8U);
            assert(m1.channels() == 3);
            double r = 0;
            const static int bar = 128;
            for (int i=0; i<m1.rows; i++)
            {
                for (int j=0; j<m1.cols; j++)
                {
                    cv::Vec3b m1ij = m1.at<cv::Vec3b>(i, j);
                    cv::Vec3b m2ij = m2.at<cv::Vec3b>(i, j);
                    r += (m1ij[0]-offset[0])*(m2ij[0]-offset[0]);
                    r += (m1ij[1]-offset[1])*(m2ij[1]-offset[1]);
                    r += (m1ij[2]-offset[2])*(m2ij[2]-offset[2]);
                }
            }
            return r;
        }
        
        double ForegroundDetector::dotproduct_(cv::Mat m1, cv::Mat m2)
        {
            assert(m1.rows == m2.rows);
            assert(m1.cols == m2.cols);
            assert(m1.depth() == m2.depth());
            assert(m1.channels() == m2.channels());
            assert(m1.depth() == CV_8U);
            assert(m1.channels() == 3);
            return sqrt(sqr_(dot_(m1, m2))/dot_(m1, m1)/dot_(m2, m2));
        }
        
        similarity_ ForegroundDetector::FilterByObject(const cv::Mat & source_mat, ObjectInfo **objs, int objnum)
        {
            //copy the source
            cv::Mat desk_mat;
            source_mat.copyTo(desk_mat);
            //filter by entropy
            FilterByEntropy(source_mat, desk_mat);
            //dilate, erode, and dilate again
            OpenImage(desk_mat);
            //build mask
            cv::Mat mask_mat = BuildMask(desk_mat);
            DilateImage(mask_mat, 3);
            //divide mask by y
            DivideMaskByY(mask_mat);
            //find contour again(this time divided)
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask_mat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            //find biggest contour
            double largest_area = 0.0;
            for(int i = 0; i < contours.size(); i++ )
            {
                double a = cv::contourArea(contours[i], false);
                if(a > largest_area)
                {
                    largest_area=a;
                }
            }
            //find the most similar objects
            double max_dp = similarity_threshold_;
            similarity_ s;
            s.object_no = -1;
            for (int i = 0; i < contours.size(); i++)
            {
                double a = cv::contourArea(contours[i], false);
                if (a > largest_area / 3)
                {
                    //get bound of the contour
                    cv::Rect bound = cv::boundingRect(contours[i]);
                    if(s.object_no == -1) s.pos = bound;
                    //cut out the object
                    cv::Mat obj_mat = cv::Mat(source_mat, bound);
                    ObjectInfo obj(obj_mat);
                    cv::Mat obj_mat_norm = obj.GetImage();
                    //calculate similarity
                    for (int j = 0; j < objnum; j++)
                    {
                        cv::Mat origin_mat_norm = objs[j]->GetImage();
                        double dp = dotproduct_(obj_mat_norm, origin_mat_norm);
                        if (dp > max_dp)
                        {
                            s.object_no = j;
                            s.dotproduct = dp;
                            s.pos = bound;
                            max_dp = dp;
                        }
                    }
                }
            }
            return s;
        }
        
        void ForegroundDetector::DrawRectOnDetect(const cv::Mat & source_mat, cv::Mat & desk_mat, ZhuangBiObjectInfo **objs, int objnum)
        {
            source_mat.copyTo(desk_mat);
            //dirty work :convert ZhuangBiObjectInfo** into ObjectInfo**
            tinker::vision::ObjectInfo **objs_plain = new ObjectInfo*[objnum];
            for (int i = 0; i < objnum; i++)
            {
                objs_plain[i] = (ObjectInfo *)objs[i];
            }
            similarity_ s = FilterByObject(source_mat, objs_plain, objnum);
            //zhuang bi
            if (s.object_no != -1)
            {
                ZhuangBiObjectInfo *obj = objs[s.object_no];
                ROS_INFO("find %s, probability %f", obj->GetName(), (float)s.dotproduct);
                cv::rectangle(desk_mat, s.pos, obj->GetColor());
            }
            else 
            {
                ROS_INFO("find nothing");
            }
        }
        
        cv::Mat ForegroundDetector::TakePhoto(const cv::Mat & source_mat)
        {
            cv::Mat desk_mat;
            similarity_ s = FilterByObject(source_mat, NULL, 0);
            cv::Mat roi(source_mat, s.pos);
            roi.copyTo(desk_mat);
            return desk_mat;
        }
        
    }
}

