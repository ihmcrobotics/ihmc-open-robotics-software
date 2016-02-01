// SUGGESTED TAB SIZE: 4
#ifndef _KDTREE_H_
#define _KDTREE_H_
 
#ifndef CPPONLY
	#include "mex.h"
#endif

#include <vector>    // point datatype
#include <math.h>    // fabs operation
#include "MyHeaps.h" // priority queues
#include "float.h"   // max floating point number

using namespace std;

typedef vector<double> Point;

/// The root node is stored in position 0 of nodesPtrs
#define ROOT 0

class Node{
public:
    double		key;		// the key (value along k-th dimension) of the split
    int			LIdx;		// the index to the left cell  (-1 if none)
	int			RIdx;		// the index to the right cell (-1 if none)
	
	/**
	 * A poiter back to the structure data of input points, 
	 * but ONLY if the node is a LEAF, otherwise value is (-1)
	 */	
	int			pIdx;
	
	inline bool isLeaf() const{
		return pIdx>=0;
	}
	/// Default constructor
	Node(){
		LIdx = -1;
		RIdx = -1;
		key  = -1;
		pIdx  = -1;
	}
};
class KDTree {

	// Core data contained in the tree
	private: vector<Point> points;    // Points data
	private: vector<Node*> nodesPtrs; // Memory to keep nodes
	private: int ndim;                // Data dimensionality
	private: int npoints;             // Number of points
	private: vector<int>   workarray; // Used in tree construction
	
	// Data used ONLY for "kNN" search (k-nearest neighbors)
	private: Point Bmin;  		 	  // bounding box lower bound
	private: Point Bmax;  		      // bounding box upper bound
	private: MaxHeap<double> pq;  	  // <key,idx> = <distance, node idx>
	private: int k;					  // number of records to search for
	private: bool terminate_search;   // true if k points have been found
	
    /// Default destructor (delete the nodes)
	public: ~KDTree(){
    	for (unsigned int i=0; i < nodesPtrs.size(); i++)
			delete nodesPtrs[i];
    }
    
	/// Heapsort algorithm used by fast KDtree construction
	// Note: this is copied almost verbatim from the heapsort 
	// wiki page: http://en.wikipedia.org/wiki/Heapsort 
	// 11/9/05
	// there was a bug for len==2 though!
	int heapsort(int dim, vector<int>& idx, int len) {
		unsigned int n = len;
		unsigned int i=len/2;
		unsigned int parent, child;
		int t;

//		if( len == 2 ){
//			if( points[idx[0]][dim]>points[idx[1]][dim] ){
//				t = idx[1];
//				idx[1] = idx[0];
//				idx[0] = t;
//			}
//			return 0;
//		}			
		
		for (;;) {
			if (i>0) {
				i--;
				t = idx[i];
			} else {
				n--;
				if (n ==0)
					return 0;
				t = idx.at(n);
				idx[n] = idx[0];
			}

			parent = i;
			child = i*2+1;

			while (child < n) {
				if ((child +1 < n) && (points[idx[child+1]][dim]>points[idx[child]][dim])) {
						child++;
				}
					if(points[idx[child]][dim]> points[t][dim]) {
						idx[parent] = idx[child];
						parent = child;
						child = parent*2+1;
					}
					else {
						break;
					}
			}
			idx[parent] = t;
		} // end of for loop
		return 0;
	} // end of heapsort
	
	
    /**
     * Creates a KDtree filled with the provided data.
     * 
     * @param points   a vector< vector<double> > containing the point data
     * 				   the number of points and the dimensionality is inferred 
     *                 by the data
     */
	public: KDTree(const vector<Point>& points){
    	// initialize data
    	this -> npoints   = points.size();
    	this -> ndim      = points[0].size();
        this -> points    = points;
        nodesPtrs.reserve( npoints );
        workarray.resize( npoints, -1 ); //used in sorting based construction
        
	    // create the heap structure to support the tree creation
	    vector< MinHeap<double> > heaps(ndim, npoints);
	    for( int dIdx=0; dIdx<ndim; dIdx++ )
	    	for( int pIdx=0; pIdx<npoints; pIdx++ )
	    		heaps[dIdx].push( points[pIdx][dIdx], pIdx );
		   
	    // Invoke heap sort generating indexing vectors
	    // indexes[dim][i]: in dimension dim, which is the index of the i-th smallest element?
	    vector< vector<int> > indexes( ndim, vector<int>(npoints,0) );
	    for( int dIdx=0; dIdx<ndim; dIdx++ )
	    	heaps[dIdx].heapsort( indexes[dIdx] );    
    
	    // Invert the indexing structure!! 
	    // srtidx[dim][j]: in dimension dim, which is ordering number of point j-th?
	    // the j-th smallest entry in dim dimension
	    vector< vector<int> > srtidx( ndim, vector<int>(npoints,0) );
	    for (unsigned int dim=0; dim<indexes.size(); dim++)
	    	for (unsigned int i=0; i < indexes[dim].size(); i++)
	    		srtidx[dim][indexes[dim][i]] = i;
	    
	    // DEBUG: visualize indexes
		#ifdef DEBUG
		cout << "indexing matrix" << endl;
	    for (unsigned int j=0; j <indexes.size(); j++){
	    	for (unsigned int i=0; i < indexes[j].size(); i++) 
	    		cout << indexes[j][i] << " ";
	    	cout << endl;
	    }
		cout << "sortindex matrix" << endl;
	    for (unsigned int j=0; j <srtidx.size(); j++){
	    	for (unsigned int i=0; i < srtidx[j].size(); i++) 
	    		cout << srtidx[j][i] << " ";
	    	cout << endl;
	    }
		#endif
	    
	    // First partition is on every single point ([1:npoints])
	    vector<int> pidx(npoints,0);
	    for (int i = 0; i < npoints; i++) pidx[i] = i;
	    build_recursively(srtidx, pidx, 0);
    }
	
	/// @return the number of points in the kd-tree
	public: inline int size(){ return points.size(); }

	/// @return the number of points in the kd-tree
	public: inline int ndims(){ return ndim; }
	
	/**
	 * Algorithm that recursively performs median splits along dimension "dim"
	 * using the pre-prepared information given by the sorting.
	 * 
	 * @param sortidx: the back indexes produced by sorting along every dimension used
	 *                 for linear time median computation
	 * @param pidx:    a vector of indexes to active elements
	 *
	 * @param dim:     the current split dimension
	 * 
	 * @note this is the memory-friendly version
	 */
	private: int build_recursively(vector< vector<int> >& sortidx, vector<int>& pidx, int dim){
		#ifdef DEBUG
		cout << endl << endl << "received current vector: ";
		for( unsigned int i=0; i< pidx.size(); i++ )
			cout << pidx[ i ] << " ";
		cout << endl;
		#endif
		
		// Stop condition
	  	if(pidx.size() == 1) {
	  		Node *node = new Node();		// create a new node
			int nodeIdx = nodesPtrs.size(); // its address is
			nodesPtrs.push_back( node ); 	// important to push back here
			node->LIdx = -1;				// no child
			node->RIdx = -1;    			// no child
			node->pIdx = pidx[0]; 			// the only index available
			node->key = 0;					// key is useless here
			return nodeIdx;
		}
	    
		// allocate the vectors
	  	vector<int> Larray;
	  	vector<int> Rarray;
	 	Larray.reserve( pidx.size()/2 + (pidx.size()%2==0 ? 0:1) );
		Rarray.reserve( pidx.size()/2 );

		// initialize the "partition" array
		// Setting parray to -1 indicates we are not using the point
		for (int i = 0; i < npoints; i++)
			workarray[i] = -1;
		for (unsigned int i = 0; i < pidx.size(); i++)
			workarray[ sortidx[dim][pidx[i]] ] = pidx[i];

		#ifdef DEBUG
		cout << "work array: ";
		for( unsigned int i=0; i< workarray.size(); i++ )
			cout << workarray[ i ] << " ";
		cout << endl; 
		#endif 
		
		int pivot = -1; //index of the median element
		if((double)pidx.size()*log((double)pidx.size()) < npoints) {
			Larray.resize( pidx.size()/2 + (pidx.size()%2==0 ? 0:1), -1 );
			Rarray.resize( pidx.size()/2, -1 );
			heapsort(dim,pidx,pidx.size());
			std::copy(pidx.begin(), pidx.begin()+Larray.size(), Larray.begin());
			std::copy(pidx.begin()+Larray.size(), pidx.end(),   Rarray.begin());
			pivot = pidx[(pidx.size()-1)/2];
		}
		else{
			// The middle valid value of parray is the pivot,
			// the left go to a node on the left, the right
			// and the pivot go to a node on the right.
			unsigned int TH = (pidx.size()-1)/2; //defines median offset
			unsigned int cnt = 0; //number of points found
			for (int i=0; i < npoints; i++) {
				// Is the current point not in the current selection? skip
				if (workarray[i] == -1)
					continue;
					
				// len/2 is on the "right" of pivot. 
				// Pivot is still put on the left side
				if (cnt == TH) {
					pivot = workarray[i];
					Larray.push_back( workarray[i] );
				} else if (cnt > TH)
					Rarray.push_back( workarray[i] );
				else
					Larray.push_back( workarray[i] );
					
				// Don't overwork, if we already read all the necessary just stop.
				cnt++;
				if(cnt>pidx.size())
					break;
			}
		}
		
		#ifdef DEBUG
		cout << "passing L: ";
		for( unsigned int i=0; i< Larray.size(); i++ )
			cout << Larray[ i ] << " ";
		cout << endl;
		cout << "passing R: ";
		for( unsigned int i=0; i< Rarray.size(); i++ )
			cout << Rarray[ i ] << " ";
		cout << endl;
		#endif
		
		// CREATE THE NODE
		Node* node = new Node();
		int nodeIdx = nodesPtrs.size(); //why it's not +1? should not happend after push back? -> no since size() is the index of last element+1!!
		nodesPtrs.push_back( node ); //important to push back here
		node->pIdx  	= -1; //not a leaf
		node->key  		= points[ pivot ][ dim ];
		node->LIdx 		= build_recursively( sortidx, Larray, (dim+1)%ndim );
		node->RIdx 		= build_recursively( sortidx, Rarray, (dim+1)%ndim );
		return nodeIdx;
	}
	
	/**
	 * Prints the tree traversing linearly the structure of nodes 
	 * in which the tree is stored.
	 */
	private: void linear_tree_print(){
		for (unsigned int i=0; i < nodesPtrs.size(); i++) {
			Node* n = nodesPtrs[i];
			cout << "[i]" << i << " key: " << n->key << " LIdx: "<< n->LIdx << " RIdx: "<< n->RIdx << endl; 
		}
	}
	
	/**
	 * Prints the tree in depth first order, visiting
	 * the node to the left, then the root, then the node
	 * to the right recursively.
	 * 
	 * @param nodeIdx the node of the index from which to start printing
	 *        (default is the root)
	 */ 
	public: void left_depth_first_print( int nodeIdx = 0 ){
		Node* currnode = nodesPtrs[nodeIdx];
		
		if( currnode -> LIdx != -1 )
			left_depth_first_print( currnode -> LIdx );
		cout << currnode -> key << " ";				
		if( currnode -> RIdx != -1 )
			left_depth_first_print( currnode -> RIdx );
	}
	
	/**
	 * Prints the tree in a structured way trying to make clear 
	 * the underlying hierarchical structure using indentation.
	 * 
	 * @param index the index of the node from which to start printing
	 * @param level the key-dimension of the node from which to start printing
	 */
	void print_tree( int index = 0, int level = 0 ){
		Node* currnode = nodesPtrs[index];
		
		// leaf
		if( currnode->pIdx >= 0 ){
			cout << "--- "<< currnode->pIdx+1 << " --- "; //node is given in matlab indexes
			for( int i=0; i<ndim; i++ ) cout << points[ currnode->pIdx ][ i ] << " ";
			cout << endl;
		}
		else
			cout << "l(" << level%ndim << ") - " << currnode->key << " nIdx: " << index << endl;
		
		// navigate the childs
		if( currnode -> LIdx != -1 ){
			for( int i=0; i<level; i++ ) cout << "  ";
			cout << "left: ";
			print_tree( currnode->LIdx, level+1 );
		}
		if( currnode -> RIdx != -1 ){
			for( int i=0; i<level; i++ ) cout << "  ";
			cout << "right: ";
			print_tree( currnode->RIdx, level+1 );
		}		
	}
    
	/**
	 * @param a a point in ndim-dimension
	 * @param b a point in ndim-dimension
	 * @returns L2 distance (in dimension ndim) between two points
	 */
	inline double distance_squared( const vector<double>& a, const vector<double>& b){
		double d = 0;
		double N = a.size(); 
		for( int i=0; i<N; i++ )
			d += (a[i]-b[i])*(a[i]-b[i]);
		return d;
	}
	
	/**
	 * k-NN query: computes the k closest points in the database to a given point 
	 * and returns their indexes.
	 * 
	 * @param Xq the query point
	 * @param k  the number of neighbors to search for
	 * @param idxsInRange the indexes of the points found
	 * 
	 */
	public: void k_closest_points(const Point& Xq, int k, vector<int>& idxs, vector<double>& distances){
    	// initialize search data
    	Bmin = vector<double>(ndim,-DBL_MAX);
    	Bmax = vector<double>(ndim,+DBL_MAX);
    	this->k = k;
    	this->terminate_search = false;
    	
    	// call search on the root [0] fill the queue
    	// with elements from the search
    	knn_search( Xq );
	
    	// scan the created pq and extract the first "k" elements
    	// pop the remaining
    	int N = pq.size();
    	for (int i=0; i < N; i++) {
			pair<double, int> topel = pq.top();
    		pq.pop();
    		if( i>=N-k ){
    			idxs.push_back( topel.second );
                distances.push_back( sqrt(topel.first) ); // it was distance squared
            }
		}
    	
    	// invert the vector, passing first closest results
    	std::reverse( idxs.begin(), idxs.end() );
        std::reverse( idxs.begin(), idxs.end() );
    }
	
	private: void leaves_of_node( int nodeIdx, vector<int>& indexes ){
		Node* node = nodesPtrs[ nodeIdx ];
		if( node->isLeaf() ){
			indexes.push_back( node->pIdx );
			return;
		}
		
		leaves_of_node( node->LIdx, indexes );
		leaves_of_node( node->RIdx, indexes );
	}
	
	/**
	 * The algorithm that computes kNN on a k-d tree as specified by the 
	 * referenced paper.
	 * 
	 * @param nodeIdx the node from which to start searching (default root)
	 * @param Xq the query point
	 * @param dim the dimension of the current node (default 0, the first)
	 * 
	 * @note: this function and its subfunctions make use of shared 
	 *        data declared within the data structure: Bmin, Bmax, pq
	 * 
	 * @article{friedman1977knn,
	 *          author = {Jerome H. Freidman and Jon Louis Bentley and Raphael Ari Finkel},
	 *          title = {An Algorithm for Finding Best Matches in Logarithmic Expected Time},
	 *          journal = {ACM Trans. Math. Softw.},
	 *          volume = {3},
	 *          number = {3},
	 *          year = {1977},
	 *          issn = {0098-3500},
	 *          pages = {209--226},
	 *          doi = {http://doi.acm.org/10.1145/355744.355745},
	 *          publisher = {ACM},
	 *          address = {New York, NY, USA}}
	 */
	private: void knn_search( const Point& Xq, int nodeIdx = 0, int dim = 0){
		// cout << "at node: " << nodeIdx << endl;
		Node* node = nodesPtrs[ nodeIdx ];
		double temp; 
		
		// We are in LEAF
		if( node -> isLeaf() ){
			double distance = distance_squared( Xq, points[ node->pIdx ] );

			// pqsize is at maximum size k, if overflow and current record is closer
			// pop further and insert the new one
			if( pq.size()==k && pq.top().first>distance ){
				pq.pop(); // remove farther record
				pq.push( distance, node->pIdx ); //push new one
			}
			else if( pq.size()<k )
				pq.push( distance, node->pIdx ); 

			return;
		}
		
		////// Explore the sons //////
		// recurse on closer son
		if( Xq[dim] <= node->key ){
			temp = Bmax[dim]; Bmax[dim] = node->key;
			knn_search( Xq, node->LIdx, (dim+1)%ndim );
			Bmax[dim] = temp;
		}
		else{
			temp = Bmin[dim]; Bmin[dim] = node->key;
			knn_search( Xq, node->RIdx, (dim+1)%ndim );
			Bmin[dim] = temp;
		}
		// recurse on farther son
		if( Xq[dim] <= node->key ){
			temp = Bmin[dim]; Bmin[dim] = node->key;
			if( bounds_overlap_ball(Xq) )
				knn_search( Xq, node->RIdx, (dim+1)%ndim );
			Bmin[dim] = temp;
		}	
		else{
			temp = Bmax[dim]; Bmax[dim] = node->key;
			if( bounds_overlap_ball(Xq) )
				knn_search( Xq, node->LIdx, (dim+1)%ndim );
			Bmax[dim] = temp;
		}
    }
	
    /** @see knn_search
     * this function was in the original paper implementation.
     * Was this function useful? How to implement the "done"
     * as opposed to "return" was a mistery. It was used to 
     * interrupt search. It might be worth to check its purpose.
     * 
     * Verifies if the ball centered in the query point, which 
     * radius is the distace from the sample Xq to the k-th best
     * found point, doesn't touches the boundaries of the current
     * BBox.
     * 
     * @param Xq the query point
     * @return true if the search can be safely terminated, false otherwise
     */
	private: bool ball_within_bounds(const Point& Xq){
		
    	//extract best distance from queue top
    	double best_dist = sqrt( pq.top().first ); 
    	// check if ball is completely within BBOX
    	for (int d=0; d < ndim; d++)
    		if( fabs(Xq[d]-Bmin[d]) < best_dist || fabs(Xq[d]-Bmax[d]) < best_dist )
    			return false;
    	return true;
    }
	/** @see knn_search
	 * 
	 * This is the search bounding condition. It checks wheter the ball centered
	 * in the sample point, with radius given by the k-th closest point to the query
	 * (if k-th closest not defined is \inf), touches the bounding box defined for 
	 * the current node (Bmin Bmax globals).
	 * 
	 */
	private: double bounds_overlap_ball(const Point& Xq){
		// k-closest still not found. termination test unavailable
		if( pq.size()<k )
			return true;
		
    	double sum = 0;
    	//extract best distance from queue top
    	double best_dist_sq = pq.top().first;
    	// cout << "current best dist: " << best_dist_sq << endl;
    	for (int d=0; d < ndim; d++) {
    		// lower than low boundary
    		if( Xq[d] < Bmin[d] ){
    			sum += ( Xq[d]-Bmin[d] )*( Xq[d]-Bmin[d] );
    			if( sum > best_dist_sq )
    				return false;
    		}
    		else if( Xq[d] > Bmax[d] ){
    			sum += ( Xq[d]-Bmax[d] )*( Xq[d]-Bmax[d] );
    			if( sum > best_dist_sq )
    				return false;
    		}
    		// else it's in range, thus distance 0
    	}
    	
    	return true;
    }

	/// Computes the closest point in the set to the query point "p"
	public: int closest_point(Point p){
		// search closest leaf
		int dim = 0; // variable to cycle through dimensions
		Node* leaf = nodesPtrs[0];
		for (;;) {
			// Is leaf node... this is my stop
			if( leaf->pIdx >= 0 )
				break;
			
			// Not a leaf... browse through
			if( p[dim] <= leaf->key )
				leaf = nodesPtrs[ leaf -> LIdx ];
			else
				leaf = nodesPtrs[ leaf -> RIdx ];
			dim = (dim + 1) % ndim;
		}
		
		double cdistsq = distance_squared( p, points[leaf->pIdx] ); 	// best distance at the moment 
		// cout << "first approximation: " << leaf->idx+1 << endl;
		int closest_neighbor = leaf->pIdx; 
		check_border_distance(ROOT, 0, p, cdistsq, closest_neighbor); 		//check if anything else can do better
		return closest_neighbor;
	}
	/** @ see closest_point
	 * 
	 * This function is the algorithm in support of "closest_point" for 
	 * closest point computation.
	 * 
	 * @param nodeIdx the index of the node to check for the current recursion
	 * @param dim     the dimension to check in the current recursion
	 * @param pnt     the query point
	 * @param cdistsq the euclidean distance for query to the point "idx"
	 * @param idx     the index to the "currently" valid closest point
	 */
	private: void check_border_distance(int nodeIdx, int dim, Point pnt, double& cdistsq, int& idx){
		Node* node = nodesPtrs[ nodeIdx ];
		// cout << "checking node: " << node->idx+1 << endl;
		
		// Are we at a leaf node? check if condition and close recursion
		if( node->pIdx >= 0 ){
			// is the leaf closer in distance?
			float dsq = distance_squared(pnt, points[ node->pIdx ] );
			if (dsq < cdistsq){
				cdistsq = dsq;
			    idx = node->pIdx;
			    // cout << "updated optimal with: " << node -> idx+1 << endl;
			}
			return;
		}
		
		// The distance squared along the CURRENT DIMENSION between the point and the key
		float ndistsq = (node->key - pnt[dim])*(node->key - pnt[dim]);
		// cout << "distance to key: " << ndistsq << " optimal current distance: " << cdistsq << "(dim:"<<dim<<")"<<endl;
	
		// If the distance squared from the key to the current value is greater than the 
		// nearest distance, we need only look in one direction.
		if (ndistsq > cdistsq) {
			if (node->key > pnt[dim])
				check_border_distance(node->LIdx, (dim+1)%ndim, pnt, cdistsq, idx);
		    else
		    	check_border_distance(node->RIdx, (dim+1)%ndim, pnt, cdistsq, idx);
		}
		// If the distance from the key to the current value is less than the nearest distance, 
		// we still need to look in both directions.
		else {
			//cout << "both directions need to be checked" << endl;
			check_border_distance(node->LIdx, (dim+1)%ndim, pnt, cdistsq, idx);
		    check_border_distance(node->RIdx, (dim+1)%ndim, pnt, cdistsq, idx);
		}
	}
	
	/**
	 * Query all points at distance less or than radius from point
	 * 
	 * @param point the center of the ndim dimensional query ball
	 * @param radius the radius of the ndim dimensional query ball
	 * @param idxsInRange (return) a collection of indexes of points that fall within
	 *        the given ball.
	 * @param distances the distances from the query point to the points within the ball
	 * 
	 * @note This is a fairly unefficient implementation for two reasons:
	 *       1) the range query is not implemented in its most efficient way
	 *       2) all the points in between the bbox and the ball are visited as well, then rejected
	 */
	public: void ball_query( const Point& point, const double radius, vector<int>& idxsInRange, vector<double>& distances ){
		// create pmin pmax that bound the sphere
		Point pmin(ndim,0);
		Point pmax(ndim,0);
		for (int dim=0; dim < ndim; dim++) {
			pmin[dim] = point[dim]-radius;
			pmax[dim] = point[dim]+radius;
		}
		// start from root at zero-th dimension
		ball_bbox_query( ROOT, pmin, pmax, idxsInRange, distances, point, radius*radius, 0 );
	}
	/** @see ball_query, range_query
	 * 
	 * Returns all the points withing the ball bounding box and their distances
	 * 
	 * @note this is similar to "range_query" i just replaced "lies_in_range" with "euclidean_distance"
	 */
	public: void ball_bbox_query(int nodeIdx, Point& pmin, Point& pmax, vector<int>& inrange_idxs, vector<double>& distances, const Point& point, const double& radiusSquared, int dim=0){
		Node* node = nodesPtrs[nodeIdx];

		// if it's a leaf and it lies in R
		if( node->isLeaf() ){
			double distance = distance_squared(points[node->pIdx], point);
			if( distance <= radiusSquared ){
				inrange_idxs.push_back( node->pIdx );
				distances.push_back( sqrt(distance) );
				return;
			}
		}
		else{
			if(node->key >= pmin[dim] && node->LIdx != -1 )
				ball_bbox_query( node->LIdx, pmin, pmax, inrange_idxs, distances, point, radiusSquared, (dim+1)%ndim);
			if(node->key <= pmax[dim] && node->RIdx != -1 )
				ball_bbox_query( node->RIdx, pmin, pmax, inrange_idxs, distances, point, radiusSquared, (dim+1)%ndim);
		}
	}
	
	/**
	 * k-dimensional Range query: given a bounding box in ndim dimensions specified by the parameters
	 * returns all the indexes of points within the bounding box.
	 * 
	 * @param pmin the lower corner of the bounding box
	 * @param pmax the upper corner of the bounding box
	 * @param inrange_idxs the indexes which satisfied the query, falling in the bounding box area
	 * 
	 */
	public: void range_query( const Point& pmin, const Point& pmax, vector<int>& inrange_idxs, int nodeIdx=0, int dim=0 ){
		Node* node = nodesPtrs[nodeIdx];
		//cout << "I am in: "<< nodeIdx << "which is is leaf?" << node->isLeaf() << endl;
	
		// if it's a leaf and it lies in R
		if( node->isLeaf() ){
			if( lies_in_range(points[node->pIdx], pmin, pmax) ){
				inrange_idxs.push_back( node->pIdx );
				return;
			}
		}
		else{
			if(node->key >= pmin[dim] && node->LIdx != -1 )
				range_query( pmin, pmax, inrange_idxs, node->LIdx, (dim+1)%ndim);
			if(node->key <= pmax[dim] && node->RIdx != -1 )
				range_query( pmin, pmax, inrange_idxs, node->RIdx, (dim+1)%ndim);
		}
	}
	/** @see range_query
	 * Checks if a point lies in the bounding box (defined by pMin and pMax)
	 * 
	 * @param p the point to be checked for
	 * @param pMin the lower corner of the bounding box
	 * @param pMax the upper corner of the bounding box
	 *
	 * @return true if the point lies in the box, false otherwise
	 */
	private: bool lies_in_range( const Point& p, const Point& pMin, const Point& pMax ){
		for (int dim=0; dim < ndim; dim++)
			if( p[dim]<pMin[dim] || p[dim]>pMax[dim] )
				return false;
		return true;
	}
};

#endif



