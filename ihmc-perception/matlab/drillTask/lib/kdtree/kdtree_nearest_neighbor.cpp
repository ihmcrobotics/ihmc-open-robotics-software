#include "KDTree.h"

#ifndef CPPONLY
#include "mex.h"
void retrieve_tree( const mxArray* matptr, KDTree* & tree){
    // retrieve pointer from the MX form
    double* pointer0 = mxGetPr(matptr);
    // check that I actually received something
    if( pointer0 == NULL )
        mexErrMsgTxt("vararg{1} must be a valid k-D tree pointer\n");
    // convert it to "long" datatype (good for addresses)
    long pointer1 = (long) pointer0[0];
    // convert it to "KDTree"
    tree = (KDTree*) pointer1;
    // check that I actually received something
    if( tree == NULL )
        mexErrMsgTxt("vararg{1} must be a valid k-D tree pointer\n");
    if( tree -> ndims() <= 0 )
        mexErrMsgTxt("the k-D tree must have k>0"); 
}
void retrieve_data( const mxArray* matptr, double*& data, int& npoints, int& ndims){	
	// retrieve pointer from the MX form
    data = mxGetPr(matptr);
    // check that I actually received something
    if( data == NULL )
        mexErrMsgTxt("vararg{2} must be a [kxN] matrix of data\n");
    // retrieve amount of points
    npoints = mxGetM(matptr);
    ndims   = mxGetN(matptr);
}
void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[]){
	// check number of arguments
	if( nrhs!=2 )
		mexErrMsgTxt("This function requires 2 arguments\n");
	if( !mxIsNumeric(prhs[0]) )
		mexErrMsgTxt("varargin{0} must be a valid kdtree pointer\n");
	if( !mxIsNumeric(prhs[1]) )
		mexErrMsgTxt("varargin{1} must be a query set of points\n");
		
    // retrieve the tree pointer
    KDTree* tree;
    retrieve_tree( prhs[0], tree ); 
    // retrieve the query data
    double* query_data;
    int npoints, ndims;
    retrieve_data( prhs[1], query_data, npoints, ndims );
    // printf("query size: %dx%d\n", npoints, ndims);
    
    // check dimensions
    if( ndims != tree->ndims() ) 
    	mexErrMsgTxt("vararg{1} must be a [Nxk] matrix of N points in k dimensions\n");   
    
    // npoints x 1 indexes in output
    plhs[0] = mxCreateDoubleMatrix(npoints, 1, mxREAL);
    double* indexes = mxGetPr(plhs[0]);
    // cout << "nindexes: " << mxGetM(plhs[0]) << "x" << mxGetN(plhs[0]) << endl;
    
    // execute the query FOR EVERY point storing the index
    vector< double > query(ndims,0);
    for(int i=0; i<npoints; i++)
    	for( int j=0; j<ndims; j++ ){
    		query[j]   = query_data[ i+j*npoints ];
    		indexes[i] = tree->closest_point(query)+1;
    	}
}
#endif

// C++ tests go here
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

	for (int i=0; i < N; i++)
		cout << "expected: " << i << " obtained: " << tree -> closest_point( A[i] ) << endl;

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
	
	for (int i=0; i < N; i++)
		cout << "expected: " << i << " obtained: " << tree -> closest_point( A[i] ) << endl;

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
	
	// Query 
	vector<double> p( A[15] );
//	p[0] = .3; p[1] = .6096; //10
//	p[0] = .2; p[1] = .6096; //30
//	p[0] = .6; p[1] = .65;   //27
//	p[0] = .3; p[1] = .22;   //30
	
	int idx_close1 = tree -> closest_point( p );
	cout << "closest neighbor: " << idx_close1+1 << endl;
	return 0;
}
int main(){
	return test1(); // perfect query on even tree
//	return test2(); // perfect query on odd tree
//	return test3(); // query on paper-based structure
}
