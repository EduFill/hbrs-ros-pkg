#include "svm.h"

SVM::SVM()
{
	this->_svmModel = NULL;
	this->_svmNodeXSpace = NULL;
	this->_bFeatureFileLoaded = false;
}

SVM::~SVM()
{
	free(this->_svmNodeXSpace);
	free(this->_svmModel);
}


/*
 */
void SVM::train(char* pInputSampleFileName, char* OutputModelFilename, double &dRetTrainError, double &dRetCrossValError)
{
	struct svm_parameter strSvmParameters;
	struct svm_problem strSvmProblem;
	struct svm_model *pstrSvmModel;
	const char *error_msg;
	double dCrossValError = -1;
	double dTrainError = -1;

	//set parameters
	this->setParameters(strSvmParameters);

	//read sample file
	this->read_problem(pInputSampleFileName, strSvmProblem, strSvmParameters);

	//check parameters
	error_msg = svm_check_parameter(&strSvmProblem, &strSvmParameters);

	//train model
	pstrSvmModel = svm_train(&strSvmProblem, &strSvmParameters);

	//do cross validation check
	dCrossValError = this->crossValidationSamples(strSvmProblem, strSvmParameters, 5);

	//save trained model
	svm_save_model(OutputModelFilename, pstrSvmModel);

	//test trained model with training set -> train error
	cout << "test model " << OutputModelFilename << " with the training set " << pInputSampleFileName << std::endl;
	this->test(pInputSampleFileName, OutputModelFilename, dTrainError);

	//clean up
	svm_destroy_model(pstrSvmModel);
	svm_destroy_param(&strSvmParameters);
	free(strSvmProblem.y);
	free(strSvmProblem.x);

	dRetTrainError = dTrainError;
	dRetCrossValError = dCrossValError;
}

/* @return: 	 0: success
 * 				-1: could not read file
 *				-2: format error
 *				-3: can't open model file filename_model
 */

int SVM::test(char* data_filename, char* filename_model, double &dTestError)
{
	int correct = 0;
	int total = 0;
	double error = 0;
	double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;
	struct svm_model* svmModel;
	int svm_type;
	int nr_class;
	double *prob_estimates=NULL;
	FILE *fInputTestData = NULL;
	int max_line_len = 0;
	char *line = NULL;
	int max_nr_attr = 64;
	struct svm_node *x = NULL;

	fInputTestData = fopen(data_filename,"r");
	if(fInputTestData == NULL)
	{
		fprintf(stderr,"can't open input file %s\n",data_filename);
		return -1;
	}

	if((svmModel=svm_load_model(filename_model))==0)
	{
		fprintf(stderr,"can't open model file %s\n",filename_model);
		return -3;
	}

	svm_type=svm_get_svm_type(svmModel);
	nr_class=svm_get_nr_class(svmModel);

	if (svm_type==NU_SVR || svm_type==EPSILON_SVR)
		printf("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n",svm_get_svr_probability(svmModel));
	else
	{
		int *labels=(int *) malloc(nr_class*sizeof(int));
		svm_get_labels(svmModel,labels);
		prob_estimates = (double *) malloc(nr_class*sizeof(double));
		//printf("labels");
		//for(j=0;j<nr_class;j++)
		//printf(" %d",labels[j]);
		//printf("\n");
		free(labels);
	}

	max_line_len = 1024;

	while((line = this->read_line(fInputTestData, max_line_len)) != NULL)
	{
		int i = 0;
		double target_label, predict_label;
		char *idx, *val, *label, *endptr;
		int inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0

		label = strtok(line," \t");
		target_label = strtod(label,&endptr);
		if(endptr == label)
		{
			fprintf(stderr,"Wrong input format at line %d\n", total+1);
			return -2;
		}

		x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));

		while(1)
		{
			if(i>=max_nr_attr-1)	// need one more for index = -1
			{
				max_nr_attr *= 2;
				x = (struct svm_node *) realloc(x,max_nr_attr*sizeof(struct svm_node));
			}

			idx = strtok(NULL,":");
			val = strtok(NULL," \t");

			if(val == NULL)
				break;
			errno = 0;

			x[i].index = (int) strtol(idx,&endptr,10);

			if(endptr == idx || errno != 0 || *endptr != '\0' || x[i].index <= inst_max_index)
			{
				fprintf(stderr,"Wrong input format at line %d\n", total+1);
				return -2;
			}
			else
				inst_max_index = x[i].index;

			errno = 0;
			x[i].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
			{
				fprintf(stderr,"Wrong input format at line %d\n", total+1);
				return -2;
			}

			++i;
		}
		x[i].index = -1;

		if (svm_type==C_SVC || svm_type==NU_SVC)
		{
			predict_label = svm_predict_probability(svmModel,x,prob_estimates);
			//printf("%g",predict_label);
			//for(j=0;j<nr_class;j++)
			//	printf(" %g",prob_estimates[j]);
			//printf("\n");
		}
		else
		{
			predict_label = svm_predict(svmModel,x);
			//	printf("%g\n",predict_label);
		}

		if(predict_label == target_label)
			++correct;

		error += (predict_label-target_label)*(predict_label-target_label);
		sump += predict_label;
		sumt += target_label;
		sumpp += predict_label*predict_label;
		sumtt += target_label*target_label;
		sumpt += predict_label*target_label;
		++total;

		free(line);

		free(x);
	}
	if (svm_type==NU_SVR || svm_type==EPSILON_SVR)
	{
		printf("Mean squared error = %g (regression)\n",error/total);
		printf("Squared correlation coefficient = %g (regression)\n",
			   ((total*sumpt-sump*sumt)*(total*sumpt-sump*sumt))/
			   ((total*sumpp-sump*sump)*(total*sumtt-sumt*sumt))
			   );
	}
	else
	{
		printf("Classification Accuracy = %g%% (%d/%d)\n",
			   (double)correct/total*100,correct,total);
	}

	dTestError = 1-((double)correct/(double)total);


	fclose(fInputTestData);
	free(prob_estimates);
	svm_destroy_model(svmModel);

	return 0;

}

/* @return: 	  1 positive classification (leg)
 * 				 -1 negative classification (not a leg)
 *				-99 could not open model file
 *
 */

double SVM::classify(struct svm_node *data, char* filename_model)
{
	double predict_label;
	double *prob_estimates = NULL;
	int svm_type;
	int nr_class;

	// load feature file only once
	if(!this->_bFeatureFileLoaded)
	{
		if((this->_svmModel = svm_load_model(filename_model))==0)
		{
			printf("can't open model file %s\n",filename_model);
			return -99;
		}

		this->_bFeatureFileLoaded = true;
	}

	svm_type = svm_get_svm_type(this->_svmModel);
	nr_class = svm_get_nr_class(this->_svmModel);

	prob_estimates = (double *) malloc(nr_class*sizeof(double));

	if (svm_type==C_SVC || svm_type==NU_SVC)
	{
		predict_label = svm_predict_probability(this->_svmModel,data,prob_estimates);

	//	printf("%g",predict_label);

		//for(int k=0; k < nr_class; k++)
			//printf(" %g",prob_estimates[k]);

		//printf("\n");
	}
	else
	{
		predict_label = svm_predict(this->_svmModel, data);

		printf("%g\n",predict_label);
	}

	free(data);
	free(prob_estimates);

	return predict_label;
}

/*
struct svm_node* SVM::convertLaserScanSegmentToSvmNode(StrLaserScanSegment* laserSegment)
{
	struct svm_node *features;

	features = new struct svm_node[laserSegment->unNumberOfFeatures + 1];

	features[1].index = 1;
	features[1].value = laserSegment->unNumberOfPoints;
	features[2].index = 2;
	features[2].value = laserSegment->dStandardDeviation;
	features[3].index = 3;
	features[3].value = laserSegment->dMeanAverageDeviationFromMedian;
	features[4].index = 4;
	features[4].value = laserSegment->dJumpDistanceFromPrecedingSegment;
	features[5].index = 5;
	features[5].value = laserSegment->dJumpDistanceToSucceedingSegment;
	features[6].index = 6;
	features[6].value = laserSegment->dWidth;
	features[7].index = 7;
	features[7].value = laserSegment->dLinearity;
	features[8].index = 8;
	features[8].value = laserSegment->dCircularity;
	features[9].index = 9;
	features[9].value = laserSegment->dRadius;
	features[10].index = 10;
	features[10].value = laserSegment->dBoundaryLength;
	features[11].index = 11;
	features[11].value = laserSegment->dBoundaryRegularity;
	features[12].index = 12;
	features[12].value = laserSegment->dMeanCurvature;
	features[13].index = 13;
	features[13].value = laserSegment->dDistanceToMedian;
	features[14].index = -1;

	return features;
}*/

/*@returns:  cross validation error
 *
 *
 */
double SVM::crossValidationSamples(struct svm_problem &strSvmProblem, struct svm_parameter &strParameters, int iNrfold)
{
	int i;
	int total_correct = 0;
	double total_error = 0;
	double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
	double *target = Malloc(double,strSvmProblem.l);
	double dCrossValError = -1;

	svm_cross_validation(&strSvmProblem,&strParameters,iNrfold,target);

	if(strParameters.svm_type == EPSILON_SVR ||	strParameters.svm_type == NU_SVR)
	{
		for(i=0;i<strSvmProblem.l;i++)
		{
			double y = strSvmProblem.y[i];
			double v = target[i];
			total_error += (v-y)*(v-y);
			sumv += v;
			sumy += y;
			sumvv += v*v;
			sumyy += y*y;
			sumvy += v*y;
		}

		dCrossValError = total_error/(double)strSvmProblem.l;

		printf("Cross Validation Mean squared error = %g\n", dCrossValError);
		printf("Cross Validation Squared correlation coefficient = %g\n",
			((strSvmProblem.l*sumvy-sumv*sumy)*(strSvmProblem.l*sumvy-sumv*sumy))/
			((strSvmProblem.l*sumvv-sumv*sumv)*(strSvmProblem.l*sumyy-sumy*sumy))
			);
	}
	else
	{
		for(i=0;i<strSvmProblem.l;i++)
			if(target[i] == strSvmProblem.y[i])
				++total_correct;

		dCrossValError = 1 - ((double)total_correct/(double)strSvmProblem.l);
		printf("Cross Validation Accuarcy = %lf%%\n", 100*(1-dCrossValError));
	}

	free(target);

	return dCrossValError;
}

/* @return 		   -> cross validation error
 * 				-1 -> could not open statistic_filename
 *
 */
int SVM::crossValidationParameters(char* pSampleFilename, char* pFixedTestFilename, char* pStatisticsFilename)
{
	struct svm_parameter strSvmParameters;
	struct svm_problem strSvmProblem;
	struct svm_model* pstrSvmModel;
	const char *error_msg;
	double dCrossValError = -1;
	double dFixTestSetError = -1;
	double dTrainError = -1;
	ofstream outputStatisticFile;
	char *pTempModelFilename = "tempSvmModel.tmp";
	double dGamma = -1;
	double dC = -1;
	char tempText[1000];

	//open statistic file
	outputStatisticFile.open(pStatisticsFilename, std::ios::trunc);

	if(!outputStatisticFile.is_open())
	{
		cout << "can not open statistic file " << pStatisticsFilename << std::endl;
		return -1;
	}

	//initial parameters
	this->setParameters(strSvmParameters);

	this->read_problem(pSampleFilename, strSvmProblem, strSvmParameters);
	error_msg = svm_check_parameter(&strSvmProblem, &strSvmParameters);

	outputStatisticFile << "\% " << pSampleFilename << std::endl;
	outputStatisticFile << "\% Gamma \t C \t TrainError \t CrossValError \t FixedTestError" << std::endl;
	cout 				<< "\% Gamma \t C \t TrainError \t CrossValError \t FixedTestError" << std::endl;


	for(int iExponentGamma = -25; iExponentGamma <= -2; ++iExponentGamma)
	{
		dGamma = pow(2.0, iExponentGamma);
		strSvmParameters.gamma = dGamma;

		for(int iExponentC = -1; iExponentC <= 15; ++iExponentC)
		{
			dC = pow(2.0, iExponentC);
			strSvmParameters.C = dC;

			//train
			pstrSvmModel = svm_train(&strSvmProblem, &strSvmParameters);
			svm_save_model(pTempModelFilename, pstrSvmModel);
			svm_destroy_model(pstrSvmModel);

			//test with train-set
			this->test(pSampleFilename, pTempModelFilename, dTrainError);

			//crossval
			dCrossValError = this->crossValidationSamples(strSvmProblem, strSvmParameters, 5);

			//test with fixed test-set
			this->test(pFixedTestFilename, pTempModelFilename, dFixTestSetError);

			remove(pTempModelFilename);

			sprintf(tempText, "%.15lf\t%10lf\t%10lf\t%10lf\t%10lf", dGamma, dC, dTrainError, dCrossValError, dFixTestSetError);

			outputStatisticFile << tempText << std::endl;
			cout 				<< "\n#######################################################################" << std::endl;
			cout				<< tempText << std::endl;
			cout 				<< "#######################################################################\n" << std::endl;
		}
	}

	//clean up
	outputStatisticFile.close();
	svm_destroy_param(&strSvmParameters);
	free(strSvmProblem.y);
	free(strSvmProblem.x);
	//free(pTempModelFilename);

	return 0;
}

int SVM::convertAdaBoostSampleFileToSVM(const char* inFilename, char* outFilename)
{
	char acBuffer[1000];
	char* temp_line = NULL;
	ifstream inputFile;
	ofstream outputFile;
	vector<std::string> vecTokens;

	//open files
	inputFile.open(inFilename, std::ios::in);
	outputFile.open(outFilename, std::ios::out | std::ios::trunc);

	//check if they are really opened
	if(!outputFile.is_open() || !inputFile.is_open())
		return -1;

	// run through input file
	while(!inputFile.eof())
	{
		// get next line
		inputFile.getline(acBuffer, sizeof(acBuffer));
		temp_line = (char*)malloc(sizeof(char*) * strlen(acBuffer));
		strcpy(temp_line, acBuffer);

		const std::string strString = temp_line;

		vecTokens = tokenize(strString, ',');

		// check each feature and transform it to svm-input-file format
		for(unsigned int i=0; i < vecTokens.size(); ++i)
		{
			if(vecTokens.at(i).compare("1") == 0)
				outputFile << "+1 ";

			else if(vecTokens.at(i).compare("2") == 0)
				outputFile << "-1 ";
			else if(i == vecTokens.size() - 1 )
				outputFile << i << ":" << vecTokens.at(i);
			else
				outputFile << i << ":" << vecTokens.at(i) << " ";

		}

		if(vecTokens.size() > 0)
			outputFile << (char)(10);
	}

	//close files
	inputFile.close();
	outputFile.close();
	free(temp_line);

	return 0;
}

/*	@return:	 0: reading of input-file successful
 * 				-1: error during opening input-file
 *				-2: format error in input-file
 */
int SVM::read_problem(const char *filename, struct svm_problem &strSvmProblem, struct svm_parameter &strParameters)
{
	int elements, max_index, inst_max_index, i, j;
	int max_line_len;
	FILE *fp = fopen(filename,"r");
	char *endptr;
	char *idx, *val, *label;
	char *line = NULL;

	if(fp == NULL)
	{
		fprintf(stderr,"can't open input file %s\n",filename);
		return -1;
	}

	strSvmProblem.l = 0;
	elements = 0;

	max_line_len = 1024;

	while((line = this->read_line(fp, max_line_len)) !=NULL)
	{
		char *p = strtok(line," \t"); // label

		// features
		while(1)
		{
			p = strtok(NULL," \t");
			if(p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
				break;
			++elements;
		}
		++elements;
		++strSvmProblem.l;

		free(line);
	}


	rewind(fp);

	strSvmProblem.y = Malloc(double,strSvmProblem.l);
	strSvmProblem.x = Malloc(struct svm_node *,strSvmProblem.l);
	this->_svmNodeXSpace = Malloc(struct svm_node,elements);

	max_index = 0;
	j=0;
	for(i=0;i<strSvmProblem.l;i++)
	{
		inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0
		line = this->read_line(fp,1024);
		strSvmProblem.x[i] = &this->_svmNodeXSpace[j];
		label = strtok(line," \t");
		strSvmProblem.y[i] = strtod(label,&endptr);
		if(endptr == label)
		{
			fprintf(stderr,"Wrong input format at line %d\n", i+1);
			return -2;
		}

		while(1)
		{
			idx = strtok(NULL,":");
			val = strtok(NULL," \t");

			if(val == NULL)
				break;

			errno = 0;
			this->_svmNodeXSpace[j].index = (int) strtol(idx,&endptr,10);
			if(endptr == idx || errno != 0 || *endptr != '\0' || this->_svmNodeXSpace[j].index <= inst_max_index)
			{
				fprintf(stderr,"Wrong input format at line %d\n", i+1);
				return -2;
			}
			else
				inst_max_index = this->_svmNodeXSpace[j].index;

			errno = 0;
			this->_svmNodeXSpace[j].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
			{
				fprintf(stderr,"Wrong input format at line %d\n", i+1);
				return -2;
			}

			++j;
		}

		if(inst_max_index > max_index)
			max_index = inst_max_index;
		this->_svmNodeXSpace[j++].index = -1;

		free(line);
	}

	if(strParameters.gamma == 0 && max_index > 0)
		strParameters.gamma = 1.0/max_index;

	if(strParameters.kernel_type == PRECOMPUTED)
		for(i=0;i<strSvmProblem.l;i++)
		{
			if (strSvmProblem.x[i][0].index != 0)
			{
				fprintf(stderr,"Wrong input format: first column must be 0:sample_serial_number\n");
				return -2;
			}
			if ((int)strSvmProblem.x[i][0].value <= 0 || (int)strSvmProblem.x[i][0].value > max_index)
			{
				fprintf(stderr,"Wrong input format: sample_serial_number out of range\n");
				return -2;
			}
		}

	fclose(fp);
	svm_destroy_param(&strParameters);

	return 0;
}

int SVM::setParameters(struct svm_parameter &strParameters)
{
	strParameters.svm_type = C_SVC;
	strParameters.kernel_type = RBF;	// LINEAR, POLY, RBF, SIGMOID, PRECOMPUTED
	strParameters.degree = 3;
	strParameters.gamma = ((double)(1.0/13.0));//((double)pow(2.0, -23));	// 1/num_features
	strParameters.coef0 = 0;
	strParameters.nu = 0.5;
	strParameters.cache_size = 100;
	strParameters.C = 1.0; //((double)pow(2.0, 12));
	strParameters.eps = 1e-3;
	strParameters.p = 0.1;
	strParameters.shrinking = 1;
	strParameters.probability = 0;
	strParameters.nr_weight = 0;
	strParameters.weight_label = NULL;
	strParameters.weight = NULL;

	return 0;
}

/* @returns: 	 0 -> success
 * 				-1 -> inconsistent lower/upper specification
 *				-2 -> can't open pSampleInputFile
 *				-3 -> can't allocate enough memory
 *				-4 -> can't open pOutputParameters
 *				-5 -> can't open pInputParameters
 *				-6 -> can't open pScaledSampleOutputFile
 *				-7 -> cannot use read and store of parameters simultaneously
 *
 */
int SVM::scaleSamples(char *pSampleInputFile, char* pScaledSampleOutputFile, double dLowerBound, double dUpperBound, char* pInputParameters, char* pOutputParameters)
{
	char *line = NULL;
	int max_line_len = 1024;
	double lower=-1.0,upper=1.0,y_lower,y_upper;
	int y_scaling = 0;
	double *feature_max;
	double *feature_min;
	double y_max = -DBL_MAX;
	double y_min = DBL_MAX;
	int max_index;
	long int num_nonzeros = 0;
	int i,index;
	FILE *fp, *fp_restore = NULL;

	if(!(upper > lower) )
	{
		fprintf(stderr,"inconsistent lower/upper specification\n");
		return -1;
	}

	if(pInputParameters != NULL && pOutputParameters != NULL)
	{
		fprintf(stderr,"cannot use read and store of parameters simultaneously\n");
		return -7;
	}

	fp=fopen(pSampleInputFile,"r");

	if(fp==NULL)
	{
		fprintf(stderr,"can't open file %s\n", pSampleInputFile);
		return -2;
	}

	line = (char *) malloc(max_line_len*sizeof(char));

	/* assumption: min index of attributes is 1 */
	/* pass 1: find out max index of attributes */
	max_index = 0;

	if(pInputParameters != NULL)
	{
		int idx, c;

		fp_restore = fopen(pInputParameters,"r");
		if(fp_restore==NULL)
		{
			fprintf(stderr,"can't open file %s\n", pInputParameters);
			return -5;
		}

		c = fgetc(fp_restore);
		if(c == 'y')
		{
			line = this->read_line(fp_restore, 1024);
			line = this->read_line(fp_restore, 1024);
			line = this->read_line(fp_restore, 1024);
		}
		line = this->read_line(fp_restore, 1024);
		line = this->read_line(fp_restore, 1024);

		while(fscanf(fp_restore,"%d %*f %*f\n",&idx) == 1)
			max_index = max(idx,max_index);
		rewind(fp_restore);
	}

	while((line = this->read_line(fp, 1024))!=NULL)
	{
		char *p=line;

		SKIP_TARGET

		while(sscanf(p,"%d:%*f",&index)==1)
		{
			max_index = max(max_index, index);
			SKIP_ELEMENT
			num_nonzeros++;
		}
	}
	rewind(fp);

	feature_max = (double *)malloc((max_index+1)* sizeof(double));
	feature_min = (double *)malloc((max_index+1)* sizeof(double));

	if(feature_max == NULL || feature_min == NULL)
	{
		fprintf(stderr,"can't allocate enough memory\n");
		return -3;
	}

	for(i=0;i<=max_index;i++)
	{
		feature_max[i]=-DBL_MAX;
		feature_min[i]=DBL_MAX;
	}

	/* pass 2: find out min/max value */
	while((line = this->read_line(fp, 1024))!=NULL)
	{
		char *p=line;
		int next_index=1;
		double target;
		double value;

		sscanf(p,"%lf",&target);
		y_max = max(y_max,target);
		y_min = min(y_min,target);

		SKIP_TARGET

		while(sscanf(p,"%d:%lf",&index,&value)==2)
		{
			for(i=next_index;i<index;i++)
			{
				feature_max[i]=maxFunc(feature_max[i],0);
				feature_min[i]=minFunc(feature_min[i],0);
			}

			feature_max[index]=maxFunc(feature_max[index],value);
			feature_min[index]=minFunc(feature_min[index],value);

			SKIP_ELEMENT
			next_index=index+1;
		}

		for(i=next_index;i<=max_index;i++)
		{
			feature_max[i]=maxFunc(feature_max[i],0);
			feature_min[i]=minFunc(feature_min[i],0);
		}
	}

	rewind(fp);

	/* pass 2.5: save/restore feature_min/feature_max */

	if(pInputParameters != NULL)
	{
		/* fp_restore rewinded in finding max_index */
		int idx, c;
		double fmin, fmax;

		if((c = fgetc(fp_restore)) == 'y')
		{
			fscanf(fp_restore, "%lf %lf\n", &y_lower, &y_upper);
			fscanf(fp_restore, "%lf %lf\n", &y_min, &y_max);
			y_scaling = 1;
		}
		else
			ungetc(c, fp_restore);

		if (fgetc(fp_restore) == 'x') {
			fscanf(fp_restore, "%lf %lf\n", &lower, &upper);
			while(fscanf(fp_restore,"%d %lf %lf\n",&idx,&fmin,&fmax)==3)
			{
				if(idx<=max_index)
				{
					feature_min[idx] = fmin;
					feature_max[idx] = fmax;
				}
			}
		}
		fclose(fp_restore);
	}

	if(pOutputParameters != NULL)
	{
		FILE *fp_save = fopen(pOutputParameters,"w");
		if(fp_save==NULL)
		{
			fprintf(stderr,"can't open file %s\n", pOutputParameters);
			exit(1);
		}
		if(y_scaling)
		{
			fprintf(fp_save, "y\n");
			fprintf(fp_save, "%.16g %.16g\n", y_lower, y_upper);
			fprintf(fp_save, "%.16g %.16g\n", y_min, y_max);
		}
		fprintf(fp_save, "x\n");
		fprintf(fp_save, "%.16g %.16g\n", lower, upper);
		for(i=1;i<=max_index;i++)
		{
			if(feature_min[i]!=feature_max[i])
				fprintf(fp_save,"%d %.16g %.16g\n",i,feature_min[i],feature_max[i]);
		}
		fclose(fp_save);
	}

	/* pass 3: scale */
	FILE *fp_scaledOutput = fopen(pScaledSampleOutputFile,"w");
	if(fp_scaledOutput==NULL)
	{
		fprintf(stderr,"can't open file %s\n", pScaledSampleOutputFile);
		return -6;
	}

	while((line = this->read_line(fp,1024))!=NULL)
	{
		char *p=line;
		int next_index=1;
		double target;
		double value;

		sscanf(p,"%lf",&target);
		fprintf(fp_scaledOutput, "%s", this->output_target(target, y_scaling, y_lower, y_upper, y_min, y_max));

		SKIP_TARGET

		while(sscanf(p,"%d:%lf",&index,&value)==2)
		{
			for(i=next_index;i<index;i++)
				fprintf(fp_scaledOutput, "%s", this->output(i,0, feature_min, feature_max, lower, upper));

			fprintf(fp_scaledOutput, "%s", this->output(index, value, feature_min, feature_max, lower, upper));

			SKIP_ELEMENT
			next_index=index+1;
		}

		for(i=next_index;i<=max_index;i++)
			fprintf(fp_scaledOutput, "%s", this->output(i, 0, feature_min, feature_max, lower, upper));

		fprintf(fp_scaledOutput, "\n");
	}

	/*
	if (new_num_nonzeros > num_nonzeros)
		fprintf(stderr,
			"Warning: original #nonzeros %ld\n"
			"         new      #nonzeros %ld\n"
			"Use -l 0 if many original feature values are zeros\n",
			num_nonzeros, new_num_nonzeros);
			*/

	free(line);
	free(feature_max);
	free(feature_min);
	fclose(fp);
	fclose(fp_scaledOutput);
	return 0;
}

char* SVM::output_target(double value, int y_scaling, double y_lower, double y_upper, double y_min, double y_max)
{
	char *output = (char*)malloc(sizeof("%g ") + sizeof(double) + 1 );

	if(y_scaling)
	{
		if(value == y_min)
			value = y_lower;
		else if(value == y_max)
			value = y_upper;
		else value = y_lower + (y_upper-y_lower) *
				 (value - y_min)/(y_max-y_min);
	}

	sprintf(output, "%g ",value);

	return output;
}

char* SVM::output(int index, double value, double *feature_min, double *feature_max, double lower, double upper)
{
	char *output = (char*)malloc(sizeof("%d:%g ") + sizeof(double) + sizeof(double) + 1 );

	strcpy(output, "");

	/* skip single-valued attribute */
	if(feature_max[index] == feature_min[index])
		return "";

	if(value == feature_min[index])
		value = lower;
	else if(value == feature_max[index])
		value = upper;
	else
		value = lower + (upper-lower) *
			(value-feature_min[index])/
			(feature_max[index]-feature_min[index]);

	if(value != 0)
	{
		sprintf(output, "%d:%g ",index, value);
		//new_num_nonzeros++;
	}

	return output;
}

char* SVM::read_line(FILE *input, int unMaxLineLength)
{
	char *line = NULL;
	int len;

	line = (char *)malloc((unMaxLineLength)*sizeof(char));

	if(fgets(line,unMaxLineLength,input) == NULL)
		return NULL;

	while(strrchr(line,'\n') == NULL)
	{
		unMaxLineLength *= 2;
		line = (char *) realloc(line,unMaxLineLength);
		len = (int) strlen(line);
		if(fgets(line+len,unMaxLineLength-len,input) == NULL)
			break;
	}

	return line;
}

std::vector<std::string> SVM::tokenize(const std::string& str, char delim)
{
    std::vector<std::string> tokens;
    std::stringstream mySstream(str);
    std::string temp;

    while(getline(mySstream, temp, delim))
        tokens.push_back(temp);

    return tokens;
}
