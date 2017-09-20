#include "KDTree.h"

#ifndef CPPONLY
#include "mex.h"
// matlab entry point
void retrieve_data( const mxArray* matptr, vector< vector<double> >& dataV, int& npoints, int& ndims){
    // retrieve pointer from the MX form
    double* data = mxGetPr(matptr);
    // check that I actually received something
    if( data == NULL )
        mexErrMsgTxt("vararg{2} must be a [kxN] matrix of data\n");
    
    // retrieve amount of points
    npoints = mxGetM(matptr);
    ndims   = mxGetN(matptr);
 
    // FILL THE DATA STRUCTURES
	dataV.resize(npoints, vector<double>(ndims));
	for( int i=0; i<npoints; i++ )
		for( int j=0; j<ndims; j++ )
			dataV[i][j] = data[ i + j*npoints ]; 
}
void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]){   
	// check input
	if( nrhs != 1 || !mxIsNumeric(prhs[0]) )
		mexErrMsgTxt("A unique [kxN] matrix of points should be passed.\n");
	
	// retrieve the data
    vector< vector<double> > input_data;
    int npoints;
    int ndims;
    retrieve_data( prhs[0], input_data, npoints, ndims );
    // printf("npoints %d ndims %d\n", npoints, ndims);
    
    // fill the k-D tree
	KDTree* tree = new KDTree( input_data );	
	
	// DEBUG
 	//mexPrintf("npoint %d dimensions %d\n", (int)input_data.size(), (int)input_data[0].size());
	
	// DEBUG
	//for (unsigned int i=0; i < input_data.size(); i++){
	//	for (unsigned int j=0; j < input_data[i].size(); j++)
	//		mexPrintf("%.2f ", input_data[i][j] );
	//	mexPrintf("\n");
	//}

    // return the program a pointer to the created tree
    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double* pointer_to_tree = mxGetPr(plhs[0]);
    pointer_to_tree[0] = (long) tree;
}
#endif

using namespace std;
#include <iostream>
int test0(){
	vector< Point > A(8, vector<double>(3,0));
	A[0][0] = 0; A[0][1] = 0; A[0][2] = 0;
	A[1][0] = 1; A[1][1] = 1; A[1][2] = 1;
	A[2][0] = 2; A[2][1] = 2; A[2][2] = 2;
	A[3][0] = 3; A[3][1] = 3; A[3][2] = 3;
	A[4][0] = 4; A[4][1] = 4;  A[4][2] = 4;
	A[5][0] = 5; A[5][1] = 5; A[5][2] = 5;
	A[6][0] = 6; A[6][1] = 6;  A[6][2] = 6;
	A[7][0] = 7; A[7][1] = 7;  A[7][2] = 7;
	KDTree* tree = new KDTree( A );
	tree -> print_tree();
	return 0;
}
int test1(){
	int N = 8;
	vector< Point > A(N, vector<double>(3,0));
	A[0][0] = 200; A[0][1] = 300; A[0][2] = 400;
	A[1][0] = 100; A[1][1] = 200; A[1][2] = 300;
	A[2][0] = 600; A[2][1] = 100; A[2][2] = 500;
	A[3][0] = 100; A[3][1] = 100; A[3][2] = 300;
	A[4][0] = 700; A[4][1] = 50;  A[4][2] = 300;
	A[5][0] = 700; A[5][1] = 110; A[5][2] = 500;
	A[6][0] = 699; A[6][1] = 51;  A[6][2] = 301;
	A[7][0] = 701; A[7][1] = 50;  A[7][2] = 305;
	KDTree* tree = new KDTree( A );
	tree -> print_tree();
	return 0;
}
int test2(){
	int N = 7;
	vector< Point > A(N, vector<double>(3,0));
	A[0][0] = 200; A[0][1] = 300; A[0][2] = 400;
	A[1][0] = 100; A[1][1] = 200; A[1][2] = 300;
	A[2][0] = 600; A[2][1] = 100; A[2][2] = 500;
	A[3][0] = 100; A[3][1] = 100; A[3][2] = 300;
	A[4][0] = 700; A[4][1] = 50;  A[4][2] = 300;
	A[5][0] = 700; A[5][1] = 110; A[5][2] = 500;
	A[6][0] = 699; A[6][1] = 51;  A[6][2] = 301;
	KDTree* tree = new KDTree( A );
	tree -> print_tree();
	return 0;
}
int test3(){
	int N = 30;
	vector< Point > A(N, vector<double>(2,0));
	A[0][0]   =  0.0565 ;  A[0][1]= 0.8670   ;
	A[1][0]   =  0.2016 ;  A[1][1]= 0.7061   ;
	A[2][0]   =  0.0795 ;  A[2][1]= 0.6096   ;
	A[3][0]   =  0.1094 ;  A[3][1]= 0.3056   ;
	A[4][0]   =  0.3583 ;  A[4][1]= 0.2529   ;
	A[5][0]   =  0.2500 ;  A[5][1]= 0.1623   ;
	A[6][0]   =  0.4919 ;  A[6][1]= 0.1477   ;
	A[7][0]   =  0.5219 ;  A[7][1]= 0.3085   ;
	A[8][0]   =  0.5620 ;  A[8][1]= 0.4635    ;
	A[9][0]  =   0.3744 ;  A[9][1] =0.6623    ;
	A[10][0]  =   0.3306 ;  A[10][1] =0.9050    ;
	A[11][0]  =   0.5081 ;  A[11][1] =0.9137    ;
	A[12][0]  =   0.4459 ;  A[12][1] =0.8699    ;
	A[13][0]  =   0.7108 ;  A[13][1] =0.6798    ;
	A[14][0]  =   0.7131 ;  A[14][1] =0.8816    ;
	A[15][0]  =   0.6279 ;  A[15][1] =0.8757    ;
	A[16][0]  =   0.6901 ;  A[16][1] =0.7675   ;
	A[17][0]  =   0.8353 ;  A[17][1] =0.7529   ;
	A[18][0]  =   0.6486 ;  A[18][1] =0.6038   ;
	A[19][0]  =   0.8145 ;  A[19][1] =0.4108   ;
	A[20][0]  =   0.6786 ;  A[20][1] =0.3319   ;
	A[21][0]  =   0.6555 ;  A[21][1] =0.4576   ;
	A[22][0]  =   0.5035 ;  A[22][1] =0.5658   ;
	A[23][0]  =   0.4412 ;  A[23][1] =0.4605   ;
	A[24][0]  =   0.4804 ;  A[24][1] =0.4137   ;
	A[25][0]  =   0.7362 ;  A[25][1] =0.3991   ;
	A[26][0]  =   0.6141 ;  A[26][1] =0.6652   ;
	A[27][0]   =  0.3698 ;  A[27][1] =0.7529   ;
	A[28][0]   =  0.2385 ;  A[28][1] =0.7822   ;
	A[29][0]   =  0.1694 ;  A[29][1] =0.6915   ;
	
	KDTree* tree = new KDTree( A );
	tree -> print_tree();
	return 0;
}
int test4(){
	int N = 100000;
	vector< Point > A(N, vector<double>(3,0));
	for (int n=0; n < N; n++) {
		// create random push and updates
		A[n][0] = double(rand()) / RAND_MAX;
		A[n][1] = double(rand()) / RAND_MAX;
		A[n][2] = double(rand()) / RAND_MAX;
	}
	KDTree* tree = new KDTree( A );
	tree -> print_tree();
	return 0;
}
int test5(){
	int N = 8;
	vector< Point > A(N, vector<double>(3,0));
	A[0][0] = 100; A[0][1] = 100; A[0][2] = 100;
	A[1][0] = 200; A[1][1] = 200; A[1][2] = 200;
	A[2][0] = 300; A[2][1] = 300; A[2][2] = 300;
	A[3][0] = 400; A[3][1] = 400; A[3][2] = 400;
	A[4][0] = 500; A[4][1] = 500; A[4][2] = 500;
	A[5][0] = 600; A[5][1] = 600; A[5][2] = 600;
	A[6][0] = 700; A[6][1] = 700; A[6][2] = 700;
	A[7][0] = 800; A[7][1] = 800; A[7][2] = 800;
		
	KDTree* T1 = new KDTree( A );
	cout << "speedup tree: " << endl;
	T1->print_tree();
	return 0;	
}
int main (int argc, char * const argv[]) {
	
	//cout << "0) returned with status: " << test0() << endl; // ordered even number nodes construction
	cout << "1) returned with status: " << test1() << endl; // even number nodes construction
	//cout << "2) returned with status: " << test2() << endl; // odd  number nodes construction
	//cout << "3) returned with status: " << test3() << endl; // paper-based tree construction
	//cout << "4) returned with status: " << test4() << endl; // large scale tree construction
	//cout << "5) returned with status: " << test5() << endl; // large scale tree construction
	return 0;
}
