package us.ihmc.imageProcessing.segmentation;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.CovarianceRandomDraw;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * @author Peter Abeles
 */
public class FitNoisyGaussian2DTest {

   Random rand = new Random(234);

   /**
    * Test with no noise added
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void perfectTest() {
      double meanX = 5;
      double meanY = 10;
      DenseMatrix64F Q = new DenseMatrix64F(2,2,true,1,0.2,0.2,2);

      // create random observations
      CovarianceRandomDraw random = new CovarianceRandomDraw(rand,Q);
      DenseMatrix64F v = new DenseMatrix64F(2,1);

      FitNoisyGaussian2D alg = new FitNoisyGaussian2D(20,1e-12,0.95);

      alg.reset();
      for( int i = 0; i < 10000; i++ ) {
         CommonOps.fill(v,0);
         random.next(v);
         alg.addPoint(v.get(0)+meanX,v.get(1)+meanY);
      }

      alg.process();

      Gaussian2D_F64 found = alg.getFound();

      assertEquals(meanX,found.x,0.2);
      assertEquals(meanY,found.y,0.2);

      assertEquals(1,found.cxx,0.3);
      assertEquals(0.2,found.cxy,0.3);
      assertEquals(2,found.cyy,0.3);
   }

   /**
    * Test with a few outliers added
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void obviousOutliers() {
      double meanX = 5;
      double meanY = 10;
      DenseMatrix64F Q = new DenseMatrix64F(2,2,true,1,0.2,0.2,2);

      // create random observations
      CovarianceRandomDraw random = new CovarianceRandomDraw(rand,Q);
      DenseMatrix64F v = new DenseMatrix64F(2,1);

      FitNoisyGaussian2D alg = new FitNoisyGaussian2D(20,1e-12,0.95);

      alg.reset();
      for( int i = 0; i < 10000; i++ ) {
         CommonOps.fill(v,0);
         random.next(v);
         alg.addPoint(v.get(0)+meanX,v.get(1)+meanY);
      }

      // outliers
      alg.addPoint(20,30);
      alg.addPoint(-20,15);

      alg.process();

      Gaussian2D_F64 found = alg.getFound();

      assertEquals(meanX,found.x,0.2);
      assertEquals(meanY,found.y,0.2);

      assertEquals(1,found.cxx,0.3);
      assertEquals(0.2,found.cxy,0.3);
      assertEquals(2,found.cyy,0.5);
   }
}
