#ifndef SVM_H_
#define SVM_H_

#include <libsvm/libsvm.h>
#include <fstream>
#include <sstream>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <malloc.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <vector>

#define Malloc(type,n) 	(type *)malloc((n)*sizeof(type))
#define maxFunc(x,y) (((x)>(y))?(x):(y))
#define minFunc(x,y) (((x)<(y))?(x):(y))

#define SKIP_TARGET\
	while(isspace(*p)) ++p;\
	while(!isspace(*p)) ++p;

#define SKIP_ELEMENT\
	while(*p!=':') ++p;\
	++p;\
	while(isspace(*p)) ++p;\
	while(*p && !isspace(*p)) ++p;


using namespace std;

class SVM
{
public:
	SVM();
	~SVM();

	void train(char* pInputSampleFileName, char* OutputModelFilename, double &dRetTrainError, double &dRetCrossValError);
	int test(char* data_filename, char* filename_model, double &dTestError);
	double classify(struct svm_node *data, char* filename_model);
	double crossValidationSamples(struct svm_problem &strSvmProblem, struct svm_parameter &strParameters, int iNrfold);
	int crossValidationParameters(char* pSampleFilename, char* pFixedTestFilename, char* pStatisticsFilename);
	int convertAdaBoostSampleFileToSVM(const char* inFilename, char* outFilename);
	//struct svm_node* convertLaserScanSegmentToSvmNode(StrLaserScanSegment* laserSegment);
	int scaleSamples(char *pSampleInputFile, char* pScalesSampleOutputFile, double dLowerBound, double dUpperBound, char* pInputParameters, char* pOutputParameters);
private:

	int read_problem(const char *filename, struct svm_problem &strSvmProblem, struct svm_parameter &strParameters);
	int setParameters(struct svm_parameter &strParameters);
	char* output_target(double value, int y_scaling, double y_lower, double y_upper, double y_min, double y_max);
	char* output(int index, double value, double *feature_min, double *feature_max, double lower, double upper);
	char* read_line(FILE *input, int unMaxLineLength);
	std::vector<std::string> tokenize(const std::string& str, char delim);

	struct svm_node *_svmNodeXSpace;
	bool _bFeatureFileLoaded;
	struct svm_model* _svmModel;
};


#endif // SVM_H_
