package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.Assert;

public class ICPOptimizationControllerHelperTest
{
   @Test
   public void testTransformFromDynamicsFrame()
   {
      ICPOptimizationControllerHelper helper = new ICPOptimizationControllerHelper();

      FrameVector2D icpVelocity = new FrameVector2D(ReferenceFrame.getWorldFrame(), 0.1, 0.1);
      DMatrixRMaj gainsToPack = new DMatrixRMaj(2, 2);
      helper.transformFromDynamicsFrame(gainsToPack, icpVelocity, 1.5, 2.5);
      double expectedX = 2.0;
      double expectedY = 2.0;

      Assert.assertEquals(expectedX, gainsToPack.get(0, 0), 1e-7);
      Assert.assertEquals(expectedY, gainsToPack.get(1, 1), 1e-7);

      icpVelocity = new FrameVector2D(ReferenceFrame.getWorldFrame(), 0.1, 0.2);
      helper.transformFromDynamicsFrame(gainsToPack, icpVelocity, 1.5, 2.5);
      expectedX = 2.3;
      expectedY = 1.7;

      Assert.assertEquals(expectedX, gainsToPack.get(0, 0), 1e-7);
      Assert.assertEquals(expectedY, gainsToPack.get(1, 1), 1e-7);

      icpVelocity = new FrameVector2D(ReferenceFrame.getWorldFrame(), 0.2, 0.1);
      helper.transformFromDynamicsFrame(gainsToPack, icpVelocity, 1.5, 2.5);
      expectedX = 1.7;
      expectedY = 2.3;

      Assert.assertEquals(expectedX, gainsToPack.get(0, 0), 1e-7);
      Assert.assertEquals(expectedY, gainsToPack.get(1, 1), 1e-7);

      icpVelocity = new FrameVector2D(ReferenceFrame.getWorldFrame(), 0.1, 0.0);
      helper.transformFromDynamicsFrame(gainsToPack, icpVelocity, 1.5, 2.5);
      expectedX = 1.5;
      expectedY = 2.5;

      Assert.assertEquals(expectedX, gainsToPack.get(0, 0), 1e-7);
      Assert.assertEquals(expectedY, gainsToPack.get(1, 1), 1e-7);

      icpVelocity = new FrameVector2D(ReferenceFrame.getWorldFrame(), 0.0, 0.1);
      helper.transformFromDynamicsFrame(gainsToPack, icpVelocity, 1.5, 2.5);
      expectedX = 2.5;
      expectedY = 1.5;

      Assert.assertEquals(expectedX, gainsToPack.get(0, 0), 1e-7);
      Assert.assertEquals(expectedY, gainsToPack.get(1, 1), 1e-7);

      icpVelocity = new FrameVector2D(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      helper.transformFromDynamicsFrame(gainsToPack, icpVelocity, 1.5, 2.5);
      expectedX = 2.5;
      expectedY = 2.5;

      Assert.assertEquals(expectedX, gainsToPack.get(0, 0), 1e-7);
      Assert.assertEquals(expectedY, gainsToPack.get(1, 1), 1e-7);
   }

}
