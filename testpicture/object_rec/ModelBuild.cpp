#include "SparseRecognition.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <opencv2/core/core_c.h>

using std::string;

int Usage();

int main(int argc, char * argv[])
{
    if(argc != 4)
        return Usage();
    string sample_dir_name(argv[1]);
	string train_sample_list_file(argv[2]);
	int n_subject_samples = atoi(argv[3]);
	const int sample_width= 80;
	const int sample_height= 100;
	CvSize sample_size = cvSize(sample_width, sample_height);
	const string src_model_file = sample_dir_name + "/sample.sr";
	vector<string> train_sample_list;
	LoadSampleList(train_sample_list_file, &train_sample_list);
	std::cout<<train_sample_list.size()<<std::endl;
	assert(train_sample_list.size() % n_subject_samples == 0);
	std::cout<<"image file loaded"<<std::endl;
	
	SRCModel *src = TrainSRCModel(train_sample_list, sample_size, n_subject_samples);
	SaveSRCModel(src, src_model_file);
	ReleaseSRCModel(&src);	
	printf("saveing model finished\n");
	return 0;

}

int Usage()
{
    printf("usage: modelbuild sample_dir sample_name_file num_sample_per_subject\n");
    return 0;
}
