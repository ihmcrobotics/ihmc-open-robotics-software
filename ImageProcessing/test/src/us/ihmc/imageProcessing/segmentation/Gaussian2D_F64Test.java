package us.ihmc.imageProcessing.segmentation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * @author Peter Abeles
 */
public class Gaussian2D_F64Test {

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void chisq() {
      Gaussian2D_F64 g = new Gaussian2D_F64();
      g.x = 5;
      g.y = 6;
      g.cxx = 2;
      g.cyy = 3;
      g.invertCovariance();

      assertEquals(0, g.chisq(5, 6), 1e-8);
      // test qualitative properties of chisq
      assertTrue(g.chisq(6, 6) > 0);
      assertTrue(g.chisq(6, 6) > g.chisq(5, 7));
      assertEquals(g.chisq(4,6),g.chisq(6,6),1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void invertCovariance() {
      Gaussian2D_F64 g = new Gaussian2D_F64();
      g.cxx = 2;
      g.cxy = 0.5;
      g.cyy = 3;
      g.invertCovariance();

      DenseMatrix64F A = new DenseMatrix64F(2,2,true,2,0.5,0.5,3);
      CommonOps.invert(A);

      assertEquals(A.get(0,0),g.sxx,1e-8);
      assertEquals(A.get(0,1),g.sxy,1e-8);
      assertEquals(A.get(1,0),g.sxy,1e-8);
      assertEquals(A.get(1,1),g.syy,1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void zero() {
      Gaussian2D_F64 g = new Gaussian2D_F64();
      g.x = 5;
      g.y = 6;
      g.cxx = 2;
      g.cxy = 1;
      g.cyy = 3;
      g.invertCovariance();

      g.zero();

      assertEquals(0,g.x,1e-8);
      assertEquals(0,g.y,1e-8);
      assertEquals(0,g.cxx,1e-8);
      assertEquals(0,g.cxy,1e-8);
      assertEquals(0,g.cyy,1e-8);
      assertEquals(0,g.sxx,1e-8);
      assertEquals(0,g.sxy,1e-8);
      assertEquals(0,g.syy,1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void set() {
      Gaussian2D_F64 g = new Gaussian2D_F64();
      g.x = 5;
      g.y = 6;
      g.cxx = 2;
      g.cxy = 1;
      g.cyy = 3;
      g.invertCovariance();

      Gaussian2D_F64 h = new Gaussian2D_F64();
      h.set(g);

      assertEquals(h.x,g.x,1e-8);
      assertEquals(h.y,g.y,1e-8);
      assertEquals(h.cxx,g.cxx,1e-8);
      assertEquals(h.cxy,g.cxy,1e-8);
      assertEquals(h.cyy,g.cyy,1e-8);
      assertEquals(h.sxx,g.sxx,1e-8);
      assertEquals(h.sxy,g.sxy,1e-8);
      assertEquals(h.syy,g.syy,1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void copy() {
      Gaussian2D_F64 g = new Gaussian2D_F64();
      g.x = 5;
      g.y = 6;
      g.cxx = 2;
      g.cxy = 1;
      g.cyy = 3;
      g.invertCovariance();

      Gaussian2D_F64 h = g.copy();

      assertEquals(h.x,g.x,1e-8);
      assertEquals(h.y,g.y,1e-8);
      assertEquals(h.cxx,g.cxx,1e-8);
      assertEquals(h.cxy,g.cxy,1e-8);
      assertEquals(h.cyy,g.cyy,1e-8);
      assertEquals(h.sxx,g.sxx,1e-8);
      assertEquals(h.sxy,g.sxy,1e-8);
      assertEquals(h.syy,g.syy,1e-8);
   }
}
