package us.ihmc.imageProcessing.sfm;

import boofcv.alg.geo.MultiViewOps;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.struct.Tuple2;
import boofcv.struct.calib.StereoParameters;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SpecializedOps;

import java.util.List;

/**
 * Given a homography, it selects the best candidate motion from the decomposed homography.  Also computes
 * plane parameters and transform from plane to left camera.
 *
 * @author Peter Abeles
 */
public class SelectBestHomography
{

   Se3_F64 l2r;
   StereoParameters param;

   // The estimated transform from left camera to the ground frame
   Se3_F64 groundToLeft = new Se3_F64();

   double distanceFromPlane;
   Vector3D_F64 normal = new Vector3D_F64();

   public SelectBestHomography(StereoParameters param)
   {
      this.param = param;
      l2r = param.getRightToLeft().invert(null);
   }

   public void process( DenseMatrix64F H , boolean pixels ) {

      if( pixels ) {
         // convert homography into euclidean space
         // H = K_b*H'_ab*inv(K_a)
         DenseMatrix64F Ka = PerspectiveOps.calibrationMatrix(param.getLeft(), null);
         DenseMatrix64F Kb_inv = PerspectiveOps.calibrationMatrix(param.getRight(),null);
         CommonOps.invert(Kb_inv);

         DenseMatrix64F temp = new DenseMatrix64F(3,3);
         DenseMatrix64F H_euclidean = new DenseMatrix64F(3,3);
         CommonOps.mult(Kb_inv,H,temp);
         CommonOps.mult(temp,Ka,H_euclidean);
         H = H_euclidean;
      }

      List<Tuple2<Se3_F64,Vector3D_F64>> solutions = MultiViewOps.decomposeHomography(H);

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

      computeTransform();
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

      double difference = SpecializedOps.diffNormF(A, I);

      double dot = v0.dot(v1);

      return difference + (1-dot);
   }

   private void computeTransform() {
      // make sure the normal is pointing downwards (+y) to simplify later calculations
      if( normal.y < 0 )
         normal.scale(-1);

      double rotX = Math.atan2(normal.z,normal.y);
      double rotZ = Math.atan2(normal.x,normal.y);

      ConvertRotation3D_F64.eulerToMatrix(EulerType.XYZ, rotX, 0, rotZ, groundToLeft.R);
      groundToLeft.T.set(normal);
      groundToLeft.T.scale(distanceFromPlane);
   }

   public Se3_F64 getGroundToLeft()
   {
      return groundToLeft;
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
