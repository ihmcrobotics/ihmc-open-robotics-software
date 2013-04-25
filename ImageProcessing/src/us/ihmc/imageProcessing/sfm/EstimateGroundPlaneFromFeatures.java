package us.ihmc.imageProcessing.sfm;

import boofcv.alg.geo.MultiViewOps;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.sfm.robust.DistanceHomographySq;
import boofcv.alg.sfm.robust.GenerateHomographyLinear;
import boofcv.struct.FastQueue;
import boofcv.struct.Tuple2;
import boofcv.struct.calib.StereoParameters;
import boofcv.struct.geo.AssociatedPair;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.homo.Homography2D_F64;
import georegression.struct.homo.UtilHomography;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.fitting.modelset.ransac.Ransac;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SpecializedOps;

import java.util.List;

/**
 * <p>
 * Given a set of associated point feature pairs between two stereo cameras, estimate the ground plane and compute
 * a transform from the left camera to the ground plane.  The ground plane is assumed to be the largest flat plane
 * in the image.
 * </p>
 *
 * <p>
 * The found transform will be from the left camera frame into the ground reference frame.  The origin of the ground
 * reference frame is defined as closest point on the ground to the left camera.  The ground z-axis is the projection
 * of the camera z-axis onto the ground.  Same is true fot the x-axis.  The ground y-axis is then the cross product of
 * the ground z and x axises.
 * </p>
 *
 * @author Peter Abeles
 */
public class EstimateGroundPlaneFromFeatures
{
   StereoParameters stereo;
   // transform from left to right camera
   Se3_F64 l2r;

   // The estimated transform from left camera to the ground frame
   Se3_F64 leftToGround = new Se3_F64();

   // Use RANSAC to estimate the Fundamental matrix
   ModelMatcher<Homography2D_F64,AssociatedPair> robustH;

   double distanceFromPlane;
   Vector3D_F64 normal = new Vector3D_F64();

   FastQueue<AssociatedPair> matches = new FastQueue<AssociatedPair>(AssociatedPair.class,true);

   public EstimateGroundPlaneFromFeatures(StereoParameters stereo)
   {
      this.stereo = stereo;
      l2r = stereo.getRightToLeft().invert(null);

//      stereo.print();

      GenerateHomographyLinear generateH = new GenerateHomographyLinear(true);

      // Input will be normalized coordinates, but error will be in pixels
      DistanceHomographySq errorMetric = new DistanceHomographySq();


      robustH = new Ransac<Homography2D_F64, AssociatedPair>(123123,generateH,errorMetric,500,0.2*0.2);
   }

   public boolean process( List<Point2D_F64> left , List<Point2D_F64> right ) {

      long start = System.currentTimeMillis();

      // convert point into normalized image coordinates
      matches.reset();
      for( int i = 0; i < left.size(); i++ ) {
         Point2D_F64 l = left.get(i);
         Point2D_F64 r = right.get(i);

         matches.grow().assign(l,r);
      }

      System.out.println("Convert pixel to norm: "+(System.currentTimeMillis()-start));

      // Estimate the plane's normal and the distance from the plane
      estimatePlane(matches.toList());

      System.out.println("Distance from plane: "+distanceFromPlane);
      normal.print();

      // Find a transform from camera to plane coordinate system
      computeTransform();

      return true;
   }

   /**
    * Estimate the plane's normal and the distance from the plane.  From the set of matched pairs a homography
    * is estimated.  The homography is then decomposed to find the camera baseline and a description of the plane.
    * The decomposition generates several hypotheses, but only the one which is the best match to the known camera
    * baseline is used.
    */
   private void estimatePlane(List<AssociatedPair> matches)
   {
      // RANSAC to find best homography
      if( !robustH.process(matches) )
         throw new IllegalArgumentException("Failed");

      // convert it into a Euclidean homography
      // H = K_b*H'_ab*inv(K_a)
      DenseMatrix64F Hd = UtilHomography.convert(robustH.getModel(), (DenseMatrix64F) null);

      DenseMatrix64F Ka = PerspectiveOps.calibrationMatrix(stereo.getLeft(),null);
      DenseMatrix64F Kb_inv = PerspectiveOps.calibrationMatrix(stereo.getRight(),null);
      CommonOps.invert(Kb_inv);

      DenseMatrix64F temp = new DenseMatrix64F(3,3);
      DenseMatrix64F H = new DenseMatrix64F(3,3);
      CommonOps.mult(Kb_inv,Hd,temp);
      CommonOps.mult(temp,Ka,H);

      // decompose homography to get transform
      List<Tuple2<Se3_F64,Vector3D_F64>> solutions = MultiViewOps.decomposeHomography(H);

//      for( Tuple2<Se3_F64,Vector3D_F64> l : solutions ) {
//         System.out.println("-------------");
//         l.data0.getR().print();
//         l.data0.getT().print();
//         System.out.println(l.data1);
//      }

      // Select the best solution based on how similar it is to the known camera baseline
      double bestScore = Double.MAX_VALUE;
      int bestIndex = -1;

      for( int i = 0; i < 4; i++ ) {
         Tuple2<Se3_F64,Vector3D_F64> l = solutions.get(i);

         double score = scoreHypothesis(l.getData0());

         if( score < bestScore ) {
            bestScore = score;
            bestIndex = i;
         }
      }

      Se3_F64 bestTran = solutions.get(bestIndex).data0;
      Vector3D_F64 bestNorm = solutions.get(bestIndex).data1;

      distanceFromPlane = l2r.getT().norm()/bestTran.getT().norm();
      normal.set(bestNorm);
      normal.normalize();
   }

   /**
    * Scores hypotheses based on how similar they are to the known camera transformation.  Closer to zero is better
    */
   public double scoreHypothesis( Se3_F64 hypothesis ) {
      DenseMatrix64F A = new DenseMatrix64F(3,3);
      CommonOps.multTransA(l2r.getR(), hypothesis.getR(), A);
      DenseMatrix64F I = CommonOps.identity(3);

      Vector3D_F64 v0 = l2r.getT().copy();
      Vector3D_F64 v1 = hypothesis.getT().copy();
      v0.normalize();
      v1.normalize();

      double difference = SpecializedOps.diffNormF(A,I);

      double dot = v0.dot(v1);

      return difference + (1-dot);
   }

   private void computeTransform() {
      // make sure the normal is pointing downwards (+y) to simplify later calculations
      if( normal.y < 0 )
         normal.ip_times(-1);

      double rotX = Math.atan2(normal.z,normal.y);
      double rotZ = Math.atan2(normal.x,normal.y);

      RotationMatrixGenerator.eulerXYZ(rotX,0,rotZ, leftToGround.R);
      leftToGround.T.set(normal);
      leftToGround.T.ip_times(distanceFromPlane);
   }

   public void getInliers( List<Point2D_F64> left , List<Point2D_F64> right ,
                           List<Point2D_F64> inliersLeft , List<Point2D_F64> inliersRight ) {
      int N = robustH.getMatchSet().size();

      for( int i = 0; i < N; i++ ) {
         int index = robustH.getInputIndex(i);
         inliersLeft.add( left.get(index) );
         inliersRight.add( right.get(index) );
      }
   }

   public Se3_F64 getLeftToGround()
   {
      return leftToGround;
   }

   public double getDistanceFromPlane()
   {
      return distanceFromPlane;
   }

   public Vector3D_F64 getNormal()
   {
      return normal;
   }
}
