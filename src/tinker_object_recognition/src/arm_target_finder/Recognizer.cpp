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

//Write VOC-compliant classifier results file
//-------------------------------------------
//INPUTS:
// - obj_class          The VOC object class identifier string
// - dataset            Specifies whether working with the training or test set
// - images             An array of ObdImage containing the images for which data will be saved to the result file
// - scores             A corresponding array of confidence scores given a query
// - (competition)      If specified, defines which competition the results are for (see VOC documentation - default 1)
//NOTES:
// The result file path and filename are determined automatically using m_results_directory as a base
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
    DDMParams( const string _detectorType, const string _descriptorType, const string& _matcherType ) :
        detectorType(_detectorType), descriptorType(_descriptorType), matcherType(_matcherType){}
    void read( const FileNode& fn )
    {
        fn["detectorType"] >> detectorType;
        fn["descriptorType"] >> descriptorType;
        fn["matcherType"] >> matcherType;
    }
    void write( FileStorage& fs ) const
    {
        fs << "detectorType" << detectorType;
        fs << "descriptorType" << descriptorType;
        fs << "matcherType" << matcherType;
    }
    void print() const
    {
        cout << "detectorType: " << detectorType << endl;
        cout << "descriptorType: " << descriptorType << endl;
        cout << "matcherType: " << matcherType << endl;
    }

    string detectorType;
    string descriptorType;
    string matcherType;
};

struct VocabTrainParams
{
    VocabTrainParams() : trainObjClass("cola"), vocabSize(1000), memoryUse(200), descProportion(0.3f) {}
    VocabTrainParams( const string _trainObjClass, size_t _vocabSize, size_t _memoryUse, float _descProportion ) :
            trainObjClass(_trainObjClass), vocabSize((int)_vocabSize), memoryUse((int)_memoryUse), descProportion(_descProportion) {}
    void read( const FileNode& fn )
    {
        fn["trainObjClass"] >> trainObjClass;
        fn["vocabSize"] >> vocabSize;
        fn["memoryUse"] >> memoryUse;
        fn["descProportion"] >> descProportion;
    }
    void write( FileStorage& fs ) const
    {
        fs << "trainObjClass" << trainObjClass;
        fs << "vocabSize" << vocabSize;
        fs << "memoryUse" << memoryUse;
        fs << "descProportion" << descProportion;
    }
    void print() const
    {
        cout << "trainObjClass: " << trainObjClass << endl;
        cout << "vocabSize: " << vocabSize << endl;
        cout << "memoryUse: " << memoryUse << endl;
        cout << "descProportion: " << descProportion << endl;
    }


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
    SVMTrainParamsExt( float _descPercent, float _targetRatio, bool _balanceClasses ) :
            descPercent(_descPercent), targetRatio(_targetRatio), balanceClasses(_balanceClasses) {}
    void read( const FileNode& fn )
    {
        fn["descPercent"] >> descPercent;
        fn["targetRatio"] >> targetRatio;
        fn["balanceClasses"] >> balanceClasses;
    }
    void write( FileStorage& fs ) const
    {
        fs << "descPercent" << descPercent;
        fs << "targetRatio" << targetRatio;
        fs << "balanceClasses" << balanceClasses;
    }
    void print() const
    {
        cout << "descPercent: " << descPercent << endl;
        cout << "targetRatio: " << targetRatio << endl;
        cout << "balanceClasses: " << balanceClasses << endl;
    }

    float descPercent; // Percentage of extracted descriptors to use for training.
    float targetRatio; // Try to get this ratio of positive to negative samples (minimum).
    bool balanceClasses;    // Balance class weights by number of samples in each (if true cSvmTrainTargetRatio is ignored).
};

static void readUsedParams( const FileNode& fn, string& vocName, DDMParams& ddmParams, VocabTrainParams& vocabTrainParams, SVMTrainParamsExt& svmTrainParamsExt )
{
    fn["vocName"] >> vocName;

    FileNode currFn = fn;

    currFn = fn["ddmParams"];
    ddmParams.read( currFn );

    currFn = fn["vocabTrainParams"];
    vocabTrainParams.read( currFn );

    currFn = fn["svmTrainParamsExt"];
    svmTrainParamsExt.read( currFn );
}

static void writeUsedParams( FileStorage& fs, const string& vocName, const DDMParams& ddmParams, const VocabTrainParams& vocabTrainParams, const SVMTrainParamsExt& svmTrainParamsExt )
{
    fs << "vocName" << vocName;

    fs << "ddmParams" << "{";
    ddmParams.write(fs);
    fs << "}";

    fs << "vocabTrainParams" << "{";
    vocabTrainParams.write(fs);
    fs << "}";

    fs << "svmTrainParamsExt" << "{";
    svmTrainParamsExt.write(fs);
    fs << "}";
}

static void printUsedParams( const string& vocPath, const string& resDir,
                      const DDMParams& ddmParams, const VocabTrainParams& vocabTrainParams,
                      const SVMTrainParamsExt& svmTrainParamsExt )
{
    cout << "CURRENT CONFIGURATION" << endl;
    cout << "----------------------------------------------------------------" << endl;
    cout << "vocPath: " << vocPath << endl;
    cout << "resDir: " << resDir << endl;
    cout << endl; ddmParams.print();
    cout << endl; vocabTrainParams.print();
    cout << endl; svmTrainParamsExt.print();
    cout << "----------------------------------------------------------------" << endl << endl;
}

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

static bool writeVocabulary( const string& filename, const Mat& vocabulary )
{
    cout << "Saving vocabulary..." << endl;
    FileStorage fs( filename, FileStorage::WRITE );
    if( fs.isOpened() )
    {
        fs << "vocabulary" << vocabulary;
        return true;
    }
    return false;
}

static Mat trainVocabulary( const string& filename, VocData& vocData, const VocabTrainParams& trainParams,
                     const Ptr<FeatureDetector>& fdetector, const Ptr<DescriptorExtractor>& dextractor )
{
    Mat vocabulary;
    if( !readVocabulary( filename, vocabulary) )
    {
        CV_Assert( dextractor->descriptorType() == CV_32FC1 );
        const int elemSize = CV_ELEM_SIZE(dextractor->descriptorType());
        const int descByteSize = dextractor->descriptorSize() * elemSize;
        const int bytesInMB = 1048576;
        const int maxDescCount = (trainParams.memoryUse * bytesInMB) / descByteSize; // Total number of descs to use for training.

        cout << "Extracting VOC data..." << endl;
        vector<ObdImage> images;
        vector<char> objectPresent;
        vocData.getClassImages( trainParams.trainObjClass, CV_OBD_TRAIN, images, objectPresent );

        cout << "Computing descriptors..." << endl;
        RNG& rng = theRNG();
        TermCriteria terminate_criterion;
        terminate_criterion.epsilon = FLT_EPSILON;
        BOWKMeansTrainer bowTrainer( trainParams.vocabSize, terminate_criterion, 3, KMEANS_PP_CENTERS );

        while( images.size() > 0 )
        {
            if( bowTrainer.descripotorsCount() > maxDescCount )
            {
#ifdef DEBUG_DESC_PROGRESS
                cout << "Breaking due to full memory ( descriptors count = " << bowTrainer.descripotorsCount()
                        << "; descriptor size in bytes = " << descByteSize << "; all used memory = "
                        << bowTrainer.descripotorsCount()*descByteSize << endl;
#endif
                break;
            }

            // Randomly pick an image from the dataset which hasn't yet been seen
            // and compute the descriptors from that image.
            int randImgIdx = rng( (unsigned)images.size() );
            Mat colorImage = imread( images[randImgIdx].path );
            vector<KeyPoint> imageKeypoints;
            fdetector->detect( colorImage, imageKeypoints );
            Mat imageDescriptors;
            dextractor->compute( colorImage, imageKeypoints, imageDescriptors );

            //check that there were descriptors calculated for the current image
            if( !imageDescriptors.empty() )
            {
                int descCount = imageDescriptors.rows;
                // Extract trainParams.descProportion descriptors from the image, breaking if the 'allDescriptors' matrix becomes full
                int descsToExtract = static_cast<int>(trainParams.descProportion * static_cast<float>(descCount));
                // Fill mask of used descriptors
                vector<char> usedMask( descCount, false );
                fill( usedMask.begin(), usedMask.begin() + descsToExtract, true );
                for( int i = 0; i < descCount; i++ )
                {
                    int i1 = rng(descCount), i2 = rng(descCount);
                    char tmp = usedMask[i1]; usedMask[i1] = usedMask[i2]; usedMask[i2] = tmp;
                }

                for( int i = 0; i < descCount; i++ )
                {
                    if( usedMask[i] && bowTrainer.descripotorsCount() < maxDescCount )
                        bowTrainer.add( imageDescriptors.row(i) );
                }
            }

#ifdef DEBUG_DESC_PROGRESS
            cout << images.size() << " images left, " << images[randImgIdx].id << " processed - "
                    <</* descs_extracted << "/" << image_descriptors.rows << " extracted - " << */
                    cvRound((static_cast<double>(bowTrainer.descripotorsCount())/static_cast<double>(maxDescCount))*100.0)
                    << " % memory used" << ( imageDescriptors.empty() ? " -> no descriptors extracted, skipping" : "") << endl;
#endif

            // Delete the current element from images so it is not added again
            images.erase( images.begin() + randImgIdx );
        }

        cout << "Maximum allowed descriptor count: " << maxDescCount << ", Actual descriptor count: " << bowTrainer.descripotorsCount() << endl;

        cout << "Training vocabulary..." << endl;
        vocabulary = bowTrainer.cluster();
        
        if(filename != "") // not at realtime (fucked by Du)
        {
            if( !writeVocabulary(filename, vocabulary) )
            {
                cout << "Error: file " << filename << " can not be opened to write" << endl;
                exit(-1);
            }
        }
    }
    return vocabulary;
}

static bool readBowImageDescriptor( const string& file, Mat& bowImageDescriptor )
{
    FileStorage fs( file, FileStorage::READ );
    if( fs.isOpened() )
    {
        fs["imageDescriptor"] >> bowImageDescriptor;
        return true;
    }
    return false;
}

static bool writeBowImageDescriptor( const string& file, const Mat& bowImageDescriptor )
{
    FileStorage fs( file, FileStorage::WRITE );
    if( fs.isOpened() )
    {
        fs << "imageDescriptor" << bowImageDescriptor;
        return true;
    }
    return false;
}

// Load in the bag of words vectors for a set of images, from file if possible
static void calculateImageDescriptors( const vector<ObdImage>& images, vector<Mat>& imageDescriptors,
                                Ptr<BOWImgDescriptorExtractor>& bowExtractor, const Ptr<FeatureDetector>& fdetector,
                                const string& resPath )
{
    CV_Assert( !bowExtractor->getVocabulary().empty() );
    imageDescriptors.resize( images.size() );

    for( size_t i = 0; i < images.size(); i++ )
    {
        string filename = resPath + bowImageDescriptorsDir + "/" + images[i].id + ".xml.gz";
        if( readBowImageDescriptor( filename, imageDescriptors[i] ) )
        {
#ifdef DEBUG_DESC_PROGRESS
            cout << "Loaded bag of word vector for image " << i+1 << " of " << images.size() << " (" << images[i].id << ")" << endl;
#endif
        }
        else
        {
            Mat colorImage = imread( images[i].path );
#ifdef DEBUG_DESC_PROGRESS
            cout << "Computing descriptors for image " << i+1 << " of " << images.size() << " (" << images[i].id << ")" << flush;
#endif
            vector<KeyPoint> keypoints;
            fdetector->detect( colorImage, keypoints );
#ifdef DEBUG_DESC_PROGRESS
                cout << " + generating BoW vector" << std::flush;
#endif
            bowExtractor->compute( colorImage, keypoints, imageDescriptors[i] );
#ifdef DEBUG_DESC_PROGRESS
            cout << " ...DONE " << static_cast<int>(static_cast<float>(i+1)/static_cast<float>(images.size())*100.0)
                 << " % complete" << endl;
#endif
            if( !imageDescriptors[i].empty() )
            {
                if( !writeBowImageDescriptor( filename, imageDescriptors[i] ) )
                {
                    cout << "Error: file " << filename << "can not be opened to write bow image descriptor" << endl;
                    exit(-1);
                }
            }
        }
    }
}

static void removeEmptyBowImageDescriptors( vector<ObdImage>& images, vector<Mat>& bowImageDescriptors,
                                     vector<char>& objectPresent )
{
    CV_Assert( !images.empty() );
    for( int i = (int)images.size() - 1; i >= 0; i-- )
    {
        bool res = bowImageDescriptors[i].empty();
        if( res )
        {
            cout << "Removing image " << images[i].id << " due to no descriptors..." << endl;
            images.erase( images.begin() + i );
            bowImageDescriptors.erase( bowImageDescriptors.begin() + i );
            objectPresent.erase( objectPresent.begin() + i );
        }
    }
}

static void removeBowImageDescriptorsByCount( vector<ObdImage>& images, vector<Mat> bowImageDescriptors, vector<char> objectPresent,
                                       const SVMTrainParamsExt& svmParamsExt, int descsToDelete )
{
    RNG& rng = theRNG();
    int pos_ex = (int)std::count( objectPresent.begin(), objectPresent.end(), (char)1 );
    int neg_ex = (int)std::count( objectPresent.begin(), objectPresent.end(), (char)0 );

    while( descsToDelete != 0 )
    {
        int randIdx = rng((unsigned)images.size());

        // Prefer positive training examples according to svmParamsExt.targetRatio if required
        if( objectPresent[randIdx] )
        {
            if( (static_cast<float>(pos_ex)/static_cast<float>(neg_ex+pos_ex)  < svmParamsExt.targetRatio) &&
                (neg_ex > 0) && (svmParamsExt.balanceClasses == false) )
            { continue; }
            else
            { pos_ex--; }
        }
        else
        { neg_ex--; }

        images.erase( images.begin() + randIdx );
        bowImageDescriptors.erase( bowImageDescriptors.begin() + randIdx );
        objectPresent.erase( objectPresent.begin() + randIdx );

        descsToDelete--;
    }
    CV_Assert( bowImageDescriptors.size() == objectPresent.size() );
}

static void setSVMParams( CvSVMParams& svmParams, CvMat& class_wts_cv, const Mat& responses, bool balanceClasses )
{
    int pos_ex = countNonZero(responses == 1);
    int neg_ex = countNonZero(responses == -1);
    cout << pos_ex << " positive training samples; " << neg_ex << " negative training samples" << endl;

    svmParams.svm_type = CvSVM::C_SVC;
    svmParams.kernel_type = CvSVM::RBF;
    if( balanceClasses )
    {
        Mat class_wts( 2, 1, CV_32FC1 );
        // The first training sample determines the '+1' class internally, even if it is negative,
        // so store whether this is the case so that the class weights can be reversed accordingly.
        bool reversed_classes = (responses.at<float>(0) < 0.f);
        if( reversed_classes == false )
        {
            class_wts.at<float>(0) = static_cast<float>(pos_ex)/static_cast<float>(pos_ex+neg_ex); // weighting for costs of positive class + 1 (i.e. cost of false positive - larger gives greater cost)
            class_wts.at<float>(1) = static_cast<float>(neg_ex)/static_cast<float>(pos_ex+neg_ex); // weighting for costs of negative class - 1 (i.e. cost of false negative)
        }
        else
        {
            class_wts.at<float>(0) = static_cast<float>(neg_ex)/static_cast<float>(pos_ex+neg_ex);
            class_wts.at<float>(1) = static_cast<float>(pos_ex)/static_cast<float>(pos_ex+neg_ex);
        }
        class_wts_cv = class_wts;
        svmParams.class_weights = &class_wts_cv;
    }
}

static void setSVMTrainAutoParams( CvParamGrid& c_grid, CvParamGrid& gamma_grid,
                            CvParamGrid& p_grid, CvParamGrid& nu_grid,
                            CvParamGrid& coef_grid, CvParamGrid& degree_grid )
{
    c_grid = CvSVM::get_default_grid(CvSVM::C);

    gamma_grid = CvSVM::get_default_grid(CvSVM::GAMMA);

    p_grid = CvSVM::get_default_grid(CvSVM::P);
    p_grid.step = 0;

    nu_grid = CvSVM::get_default_grid(CvSVM::NU);
    nu_grid.step = 0;

    coef_grid = CvSVM::get_default_grid(CvSVM::COEF);
    coef_grid.step = 0;

    degree_grid = CvSVM::get_default_grid(CvSVM::DEGREE);
    degree_grid.step = 0;
}

static void trainSVMClassifier( CvSVM& svm, const SVMTrainParamsExt& svmParamsExt, const string& objClassName, VocData& vocData,
                         Ptr<BOWImgDescriptorExtractor>& bowExtractor, const Ptr<FeatureDetector>& fdetector,
                         const string& resPath )
{
    /* first check if a previously trained svm for the current class has been saved to file */
    string svmFilename = resPath + svmsDir + "/" + objClassName + ".xml.gz";

    FileStorage fs( svmFilename, FileStorage::READ);
    if( fs.isOpened() )
    {
        cout << "*** LOADING SVM CLASSIFIER FOR CLASS " << objClassName << " AT " << svmFilename << " ***" << endl;
        svm.load( svmFilename.c_str() );
    }
    else
    {
        cout << "*** TRAINING CLASSIFIER FOR CLASS " << objClassName << " ***" << endl;
        cout << "CALCULATING BOW VECTORS FOR TRAINING SET OF " << objClassName << "..." << endl;

        // Get classification ground truth for images in the training set
        vector<ObdImage> images;
        vector<Mat> bowImageDescriptors;
        vector<char> objectPresent;
        vocData.getClassImages( objClassName, CV_OBD_TEST, images, objectPresent );

        // Compute the bag of words vector for each image in the training set.
        calculateImageDescriptors( images, bowImageDescriptors, bowExtractor, fdetector, resPath );

        // Remove any images for which descriptors could not be calculated
        removeEmptyBowImageDescriptors( images, bowImageDescriptors, objectPresent );

        CV_Assert( svmParamsExt.descPercent > 0.f && svmParamsExt.descPercent <= 1.f );
        if( svmParamsExt.descPercent < 1.f )
        {
            int descsToDelete = static_cast<int>(static_cast<float>(images.size())*(1.0-svmParamsExt.descPercent));

            cout << "Using " << (images.size() - descsToDelete) << " of " << images.size() <<
                    " descriptors for training (" << svmParamsExt.descPercent*100.0 << " %)" << endl;
            removeBowImageDescriptorsByCount( images, bowImageDescriptors, objectPresent, svmParamsExt, descsToDelete );
        }

        // Prepare the input matrices for SVM training.
        Mat trainData( (int)images.size(), bowExtractor->getVocabulary().rows, CV_32FC1 );
        Mat responses( (int)images.size(), 1, CV_32SC1 );
        // Transfer bag of words vectors and responses across to the training data matrices
        for( size_t imageIdx = 0; imageIdx < images.size(); imageIdx++ )
        {
            // Transfer image descriptor (bag of words vector) to training data matrix
            Mat submat = trainData.row((int)imageIdx);
            if( bowImageDescriptors[imageIdx].cols != bowExtractor->descriptorSize() )
            {
                cout << "Error: computed bow image descriptor size " << bowImageDescriptors[imageIdx].cols
                     << " differs from vocabulary size" << bowExtractor->getVocabulary().cols << endl;
                exit(-1);
            }
            bowImageDescriptors[imageIdx].copyTo( submat );

            // Set response value
            responses.at<int>((int)imageIdx) = objectPresent[imageIdx] ? 1 : -1;
        }
        cout << "TRAINING SVM FOR CLASS ..." << objClassName << "..." << endl;
        CvSVMParams svmParams;
        CvMat class_wts_cv;
        setSVMParams( svmParams, class_wts_cv, responses, false );
        svm.train_auto( trainData, responses, Mat(), Mat(), svmParams);
        cout << "SVM TRAINING FOR CLASS " << objClassName << " COMPLETED" << endl;

        svm.save( svmFilename.c_str() );
        cout << "SAVED CLASSIFIER TO FILE " << svmFilename << endl;
    }
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

    // Print configuration to screen
    printUsedParams( vocPath, resPath, ddmParams, vocabTrainParams, svmTrainParamsExt );
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
