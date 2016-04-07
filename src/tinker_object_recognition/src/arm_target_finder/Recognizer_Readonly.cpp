#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/ml/ml.hpp"

#include <fstream>
#include <iostream>
#include <memory>
#include <functional>
#include <tinker_object_recognition/tinyxml2.h>

#if defined WIN32 || defined _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef min
#undef max
#include "sys/types.h"
#endif
#include <sys/stat.h>

#define DEBUG_DESC_PROGRESS

#include "tinker_object_recognition/arm_target_finder/ForegroundDetector.h"
#include "tinker_object_recognition/utilities.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <vector>
#include <cassert>

using namespace cv;
using namespace std;
using namespace tinyxml2;

const string paramsFile = "params.xml";
const string vocabularyFile = "vocabulary.xml.gz";
const string bowImageDescriptorsDir = "/bowImageDescriptors";
const string svmsDir = "/svms";
const string plotsDir = "/plots";

static void help(char** argv)
{
    cout
    << "Format:\n ./" << argv[0] << " [VOC path] [result directory]  \n"
    << "Input parameters: \n"
    << "[VOC path]             Path to VOC data \n"
    << "[result directory]     Path to result diractory. Following folders will be created in [result directory]: \n"
    << "                         bowImageDescriptors - to store image descriptors, \n"
    << "                         svms - to store trained svms, \n"
    << "                         plots - to store files for plots creating. \n"
    << "\n";
}

static void makeDir( const string& dir )
{
#if defined WIN32 || defined _WIN32
    CreateDirectory( dir.c_str(), 0 );
#else
    mkdir( dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
#endif
}

static void makeUsedDirs( const string& rootPath )
{
    makeDir(rootPath + bowImageDescriptorsDir);
    makeDir(rootPath + svmsDir);
    makeDir(rootPath + plotsDir);
}

/****************************************************************************************\
*                            Data Set (fucked by Du on 20160401 )                        *
\****************************************************************************************/
const string objectxmlstring = "/object.xml";

//used to specify the (sub-)dataset over which operations are performed
enum ObdDatasetType {CV_OBD_TRAIN, CV_OBD_TEST};

class ObdImage
{
public:
    ObdImage(string p_id, string p_path, string p_classname) : id(p_id), path(p_path), classname(p_classname) {}
    string id;
    string path;
    string classname;
};

class VocData
{
public:
    VocData( const string& vocPath ) { initVoc( vocPath ); }
    void initVoc(const string& vocPath);
    void getClassImages(const string& obj_class, const ObdDatasetType dataset, vector<ObdImage>& images, vector<char>& object_present);
    const vector<string>& getObjectClasses() {return objectclasses_;}
    
    void writeClassifierResultsFile( const string& out_dir, const string& obj_class, const ObdDatasetType dataset, const vector<ObdImage>& images, const vector<float>& scores, const int competition, const bool overwrite_ifexists);
protected:    
    string integerToString(int input_int)
    {
        static char buffer[66];
        sprintf(buffer, "%d", input_int);
        return string(buffer);
    }
    //data members
    vector<ObdImage> images_;
    vector<string> objectclasses_;
};

void VocData::initVoc(const string& vocPath)
{
    int object_num = 0;
    XMLDocument doc;
    string docpath = vocPath + objectxmlstring;
    cout<<"vocdata xml path:"<<docpath<<endl;
    doc.LoadFile(docpath.c_str());
    XMLElement *sample = doc.FirstChildElement("sample");
    object_num = atoi(sample->FirstAttribute()->Value());
    for (XMLElement *object = sample->FirstChildElement("subject"); object;
         object = object->NextSiblingElement()) {
        const XMLAttribute *attributeOfobject = object->FirstAttribute();
        objectclasses_.push_back(attributeOfobject->Value());
    }
    int counter = 0;
    
    int subject_counter_ = 0;
    int object_counter_ = 0;
    static char buffer[66];
    while(1)
    {
        sprintf(buffer, "%s_%d.png", objectclasses_[subject_counter_].c_str(), object_counter_);
        string imgpath = vocPath + "/" + string(buffer);
        ObdImage o(integerToString(counter), imgpath, objectclasses_[subject_counter_]);
        images_.push_back(o);
        object_counter_++;
        counter++;
        if (object_counter_ == object_num) {
            object_counter_ = 0;
            subject_counter_++;
            if (subject_counter_ == objectclasses_.size()) {
                break;
            }
        }
    }
}

void VocData::getClassImages(const string& obj_class, const ObdDatasetType dataset, vector<ObdImage>& images, vector<char>& object_present)
{
    for(int i=0; i<images_.size(); i++)
    {
        ObdImage o = images_[i];
        images.push_back(o);
        object_present.push_back(o.classname == obj_class);
    }
}

void VocData::writeClassifierResultsFile( const string& out_dir, const string& obj_class, const ObdDatasetType dataset, const vector<ObdImage>& images, const vector<float>& scores, const int competition, const bool overwrite_ifexists)
{
    CV_Assert(images.size() == scores.size());

    string output_file_base, output_file;
    if (dataset == CV_OBD_TRAIN)
    {
        output_file_base = out_dir + "/comp" + integerToString(competition) + "_cls_train_" + obj_class;
    } else {
        output_file_base = out_dir + "/comp" + integerToString(competition) + "_cls_test_" + obj_class;
    }
    output_file = output_file_base + ".txt";

    //check if file exists, and if so create a numbered new file instead
    if (overwrite_ifexists == false)
    {
        struct stat stFileInfo;
        if (stat(output_file.c_str(),&stFileInfo) == 0)
        {
            string output_file_new;
            int filenum = 0;
            do
            {
                ++filenum;
                output_file_new = output_file_base + "_" + integerToString(filenum);
                output_file = output_file_new + ".txt";
            } while (stat(output_file.c_str(),&stFileInfo) == 0);
        }
    }

    //output data to file
    std::ofstream result_file(output_file.c_str());
    if (result_file.is_open())
    {
        for (size_t i = 0; i < images.size(); ++i)
        {
            result_file << images[i].id << " " << scores[i] << endl;
        }
        result_file.close();
    } else {
        string err_msg = "could not open classifier results file '" + output_file + "' for writing. Before running for the first time, a 'results' subdirectory should be created within the VOC dataset base directory. e.g. if the VOC data is stored in /VOC/VOC2010 then the path /VOC/results must be created.";
        CV_Error(CV_StsError,err_msg.c_str());
    }
}
/****************************************************************************************\
*                            Sample on image classification                             *
\****************************************************************************************/
//
// This part of the code was a little refactor
//
struct DDMParams
{
    DDMParams() : detectorType("SURF"), descriptorType("SURF"), matcherType("BruteForce") {}
    
    string detectorType;
    string descriptorType;
    string matcherType;
};

struct VocabTrainParams
{
    VocabTrainParams() : trainObjClass("cola"), vocabSize(1000), memoryUse(200), descProportion(0.3f) {}
    
    string trainObjClass; // Object class used for training visual vocabulary.
                          // It shouldn't matter which object class is specified here - visual vocab will still be the same.
    int vocabSize; //number of visual words in vocabulary to train
    int memoryUse; // Memory to preallocate (in MB) when training vocab.
                   // Change this depending on the size of the dataset/available memory.
    float descProportion; // Specifies the number of descriptors to use from each image as a proportion of the total num descs.
};

struct SVMTrainParamsExt
{
    SVMTrainParamsExt() : descPercent(1.0f), targetRatio(0.4f), balanceClasses(true) {}

    float descPercent; // Percentage of extracted descriptors to use for training.
    float targetRatio; // Try to get this ratio of positive to negative samples (minimum).
    bool balanceClasses;    // Balance class weights by number of samples in each (if true cSvmTrainTargetRatio is ignored).
};

static bool readVocabulary( const string& filename, Mat& vocabulary )
{
    cout << "Reading vocabulary...";
    FileStorage fs( filename, FileStorage::READ );
    if( fs.isOpened() )
    {
        fs["vocabulary"] >> vocabulary;
        cout << "done" << endl;
        return true;
    }
    return false;
}

static Mat trainVocabulary( const string& filename, VocData& vocData, const VocabTrainParams& trainParams,
                     const Ptr<FeatureDetector>& fdetector, const Ptr<DescriptorExtractor>& dextractor )
{
    Mat vocabulary;
    readVocabulary( filename, vocabulary )
    return vocabulary;
}
static void trainSVMClassifier( CvSVM& svm, const SVMTrainParamsExt& svmParamsExt, const string& objClassName, VocData& vocData,
                         Ptr<BOWImgDescriptorExtractor>& bowExtractor, const Ptr<FeatureDetector>& fdetector,
                         const string& resPath )
{
    /* first check if a previously trained svm for the current class has been saved to file */
    string svmFilename = resPath + svmsDir + "/" + objClassName + ".xml.gz";

    FileStorage fs( svmFilename, FileStorage::READ);
    cout << "*** LOADING SVM CLASSIFIER FOR CLASS " << objClassName << " AT " << svmFilename << " ***" << endl;
    svm.load( svmFilename.c_str() );

}

static string getVocName( const string& vocPath )
{
    cout << "path:" << vocPath << endl;
    size_t found = vocPath.rfind( '/' );
    if( found == string::npos )
    {
        found = vocPath.rfind( '\\' );
        if( found == string::npos )
            return vocPath;
    }
    return vocPath.substr(found + 1, vocPath.size() - found);
}

class predictResult
{
public:
    predictResult(string p_name = "", float p_p = 0.0f) : name(p_name), p(p_p) {}
    string name;
    float p;
};

//global variables
Mat vocabulary;
CvSVM *svms;
predictResult *pr;
Ptr<FeatureDetector> featureDetector;
Ptr<DescriptorExtractor> descExtractor;
Ptr<BOWImgDescriptorExtractor> bowExtractor; 
vector<string> objClasses;

//ros related global fucking variables
void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr &msg);
tinker::vision::ForegroundDetector fd(8, 0.265);
image_transport::Publisher pub;


int main(int argc, char** argv)
{
    if( argc != 3 )
    {
        help(argv);
        return -1;
    }

    cv::initModule_nonfree();

    const string vocPath = argv[1], resPath = argv[2];

    // Read or set default parameters
    string vocName;
    DDMParams ddmParams = DDMParams();
    VocabTrainParams vocabTrainParams = VocabTrainParams();
    SVMTrainParamsExt svmTrainParamsExt = SVMTrainParamsExt();

    // Create detector, descriptor, matcher.
    cout << "detecotrType " << ddmParams.detectorType << endl;
    cout << "descriptorType " << ddmParams.descriptorType << endl;
    featureDetector = FeatureDetector::create( ddmParams.detectorType );
    descExtractor = DescriptorExtractor::create( ddmParams.descriptorType );
    if( featureDetector.empty() || descExtractor.empty() )
    {
        cout << "featureDetector or descExtractor was not created" << endl;
        return -1;
    }
    {
        Ptr<DescriptorMatcher> descMatcher = DescriptorMatcher::create("BruteForce");
        cout << "matcherType " << ddmParams.matcherType << endl;
        if( featureDetector.empty() || descExtractor.empty() || descMatcher.empty() )
        {
            cout << "descMatcher was not created" << endl;
            return -1;
        }
        bowExtractor = new BOWImgDescriptorExtractor( descExtractor, descMatcher );
    }

    // Create object to work with VOC
    VocData vocData( vocPath);

    // 1. Train visual word vocabulary if a pre-calculated vocabulary file doesn't already exist from previous run
    vocabulary = trainVocabulary( resPath + "/" + vocabularyFile, vocData, vocabTrainParams,
                                      featureDetector, descExtractor );
    bowExtractor->setVocabulary( vocabulary );

    // 2. Train a classifier and run a sample query for each object class
    objClasses = vocData.getObjectClasses(); // object class list
    
    svms = new CvSVM[objClasses.size()];
    pr = new predictResult[objClasses.size()];
    for( size_t classIdx = 0; classIdx < objClasses.size(); ++classIdx )
    {
        // Train a classifier on train dataset
        trainSVMClassifier( svms[classIdx], svmTrainParamsExt, objClasses[classIdx], vocData,
                            bowExtractor, featureDetector, resPath );
    }
    
    //now cames ROS
    ros::init(argc, argv, "arm_webcam_object_classifier");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pub = it.advertise("tk2_com/find", 1);
    ros::Subscriber sub =
        nh.subscribe("tk2_com/arm_cam_image", 1, ArmCamImgCallback);
    ros::spin();
    
    
    delete[] svms;
    delete[] pr;
    
    return 0;
}

void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr &msg) {
    // get foreground
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat cam_mat = cv_ptr->image;
    cv::Mat res_mat;
    int find = fd.CutForegroundOut(cam_mat, res_mat);
    if (find == tinker::vision::ForegroundDetector::DETECTED) {
        // publish it
        sensor_msgs::Image pubmsg;
        cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", res_mat);
        cvi.toImageMsg(pubmsg);
        pub.publish(pubmsg);

        /****************************************************************************************\
        *                                      use SVM                                           *
        \****************************************************************************************/
        for(int i=0; i<1; i++)
        {
            Mat colorImage = res_mat;
            
            vector<KeyPoint> imageKeypoints;
            featureDetector->detect( colorImage, imageKeypoints );
            Mat imageDescriptors;
            
            bowExtractor->compute(colorImage, imageKeypoints, imageDescriptors);
            
            /****************************************************************************************\
            *                                       predict                                         *
            \****************************************************************************************/
             
            for(int clsIdx=0; clsIdx<objClasses.size(); clsIdx++)
            {
                pr[clsIdx].name = objClasses[clsIdx];
                pr[clsIdx].p = svms[clsIdx].predict(imageDescriptors);
            }
            
            float maxp = 0.0f;
            string maxp_name;
            for(int clsIdx=0; clsIdx<objClasses.size(); clsIdx++)
            {
                if(maxp < pr[clsIdx].p)
                {
                    maxp = pr[clsIdx].p;
                    maxp_name = pr[clsIdx].name;
                }
            }
            cout<<"the name is "<<maxp_name<<", and the p is "<<maxp<<endl;
        }
    }
}
