#ifndef __TINKER_VISION_BOW_DATA_H__
#define __TINKER_VISION_BOW_DATA_H__

#include <string>
#include <vector>
#include <cstdio>
#include "tinker_object_recognition/common.h"

namespace tinker {
namespace vision {

// used to specify the (sub-)dataset over which operations are performed
enum ObdDatasetType { CV_OBD_TRAIN, CV_OBD_TEST };

class ObdImage {
public:
    ObdImage(std::string p_id, std::string p_path, std::string p_classname)
        : id(p_id), path(p_path), classname(p_classname) {}
    std::string id;
    std::string path;
    std::string classname;
};

class BoWData {
public:
    BoWData() {}
    BoWData(XmlRpc::XmlRpcValue& data_info) {
        InitBoWData(data_info);
    }
    void InitBoWData(XmlRpc::XmlRpcValue& data_info);
    void GetClassImages(const std::string& obj_class,
                        const ObdDatasetType dataset,
                        std::vector<ObdImage>& images,
                        std::vector<char>& object_present);
    const std::vector<std::string> GetObjectClasses() {
        return objectclasses_;
    }

protected:
    std::string IntegerToString(int input_int) {
        static char buffer[66];
        sprintf(buffer, "%d", input_int);
        return std::string(buffer);
    }
    // data members
    std::vector<ObdImage> images_;
    std::vector<std::string> objectclasses_;
};
}
}

#endif
