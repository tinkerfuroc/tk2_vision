#include <cstdio>
#include <cstdlib>
#include <string>
#include "SparseRecognition.h"

using std::string;

int Usage();

int main(int argc, char * argv[])
{
    if(argc != 3)
        return Usage();
    string model_filename = string(argv[1]);
    string filename = string(argv[2]);
    const double threshold = 0;
    SRCModel * src_model = LoadSRCModel(model_filename);
    CvMat * y = LoadSample(filename, src_model->sample_size_);
    string name = Recognize(src_model, y, threshold, NULL, NULL);
    printf(", and the name of the subject is %s\n", name.c_str());
	ReleaseSRCModel(&src_model);	
    return 0;
}


int Usage()
{
    printf("usage: object_classify model_file sample_file\n");
    return 0;
}
