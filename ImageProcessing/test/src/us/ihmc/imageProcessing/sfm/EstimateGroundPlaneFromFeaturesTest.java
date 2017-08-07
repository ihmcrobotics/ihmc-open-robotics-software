package us.ihmc.imageProcessing.sfm;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ddogleg.struct.FastQueue;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.junit.Test;

import boofcv.alg.geo.PerspectiveOps;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.calib.StereoParameters;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * @author Peter Abeles
 */
public class EstimateGroundPlaneFromFeaturesTest
{

   /**
    * Create a set of observations with perfect data and see if it can reconstruct the plane and transform
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void perfect()
   {
      StereoParameters param = new StereoParameters();
      param.left = new IntrinsicParameters();
      param.right = new IntrinsicParameters();
      param.left.width = param.left.height = 800;
      param.left.fx = param.left.fy = 476.701;
      param.left.skew = 0;
      param.right.set(param.left);
      param.rightToLeft = new Se3_F64();
      param.getRightToLeft().getT().set(0.07,0,0);

      EstimateGroundPlaneFromFeatures groundAlg = new EstimateGroundPlaneFromFeatures(param);

      FastQueue<Point2D_F64> matchA = new FastQueue<Point2D_F64>(Point2D_F64.class,true);
      FastQueue<Point2D_F64> matchB = new FastQueue<Point2D_F64>(Point2D_F64.class,true);

      Random rand = new Random(234);

      // compute transform used when rendering
      Se3_F64 l2l = new Se3_F64();
      Se3_F64 l2r = param.getRightToLeft().invert(null);
      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(param.getLeft(), null);

      // rotate the plane.  rotation from plane reference into camera reference
      DenseMatrix64F R = ConvertRotation3D_F64.eulerToMatrix(EulerType.XYZ, 0.2, 0, 0, null);

      for( int i = 0; i < 100; i++ ) {
         Point3D_F64 X = new Point3D_F64();
         X.x = rand.nextGaussian();
         X.z = rand.nextGaussian()+2;
         X.y = 1.5;

         Point3D_F64 XX = new Point3D_F64();

         GeometryMath_F64.mult(R, X, XX);

         Point2D_F64 pa = PerspectiveOps.renderPixel(l2l,K,XX);
         Point2D_F64 pb = PerspectiveOps.renderPixel(l2r,K,XX);

         if( pa != null && pb != null ) {
            matchA.grow().set(pa);
            matchB.grow().set(pb);
         }
      }

      // make sure most of the points were visible
      assertTrue(matchA.size>90);

      groundAlg.process(matchA.toList(),matchB.toList());

      // see if it found the plane's parameters correctly
      Vector3D_F64 expectedN = new Vector3D_F64();
      expectedN.x = R.get(0,1);
      expectedN.y = R.get(1,1);
      expectedN.z = R.get(2,1);

      Vector3D_F64 foundN = groundAlg.getNormal();

      assertEquals(1.5,groundAlg.getDistanceFromPlane(),1e-8);
      assertEquals(expectedN.x,foundN.x,1e-8);
      assertEquals(expectedN.y,foundN.y,1e-8);
      assertEquals(expectedN.z,foundN.z,1e-8);

      Vector3D_F64 expectedT = new Vector3D_F64(0,1.5,0);
      GeometryMath_F64.mult(R, expectedT, expectedT);

      // check the rotation matrix
      Se3_F64 found = groundAlg.getGroundToLeft();
      assertEquals(expectedT.x , found.T.x,1e-8);
      assertEquals(expectedT.y , found.T.y,1e-8);
      assertEquals(expectedT.z , found.T.z,1e-8);

      DenseMatrix64F A = new DenseMatrix64F(3,3);
      CommonOps.multTransA(found.getR(),R,A);
      assertTrue(MatrixFeatures.isIdentity(A, 1e-8));
   }

}
