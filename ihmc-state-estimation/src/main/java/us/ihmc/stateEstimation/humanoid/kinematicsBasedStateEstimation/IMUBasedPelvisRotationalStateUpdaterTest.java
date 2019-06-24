package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class IMUBasedPelvisRotationalStateUpdaterTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testComputeOrientationAtEstimateFrame()
   {
      Random random = new Random(165415);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < 100; i++)
      {
         ReferenceFrame rootJointFrame = EuclidFrameRandomTools.nextReferenceFrame("rootJointFrame" + i, random, worldFrame);
         ReferenceFrame measurementFrame = EuclidFrameRandomTools.nextReferenceFrame("measurementFrame" + i, random, rootJointFrame);
         RotationMatrix expectedOrientation = new RotationMatrix(rootJointFrame.getTransformToRoot().getRotationMatrix());

         RotationMatrix imuOrientationMeasurement = new RotationMatrix(measurementFrame.getTransformToRoot().getRotationMatrix());

         RotationMatrix actualOrientation = new RotationMatrix();
         IMUBasedPelvisRotationalStateUpdater.computeOrientationAtEstimateFrame(measurementFrame, imuOrientationMeasurement, rootJointFrame, actualOrientation);

         EuclidCoreTestTools.assertMatrix3DEquals(expectedOrientation, actualOrientation, EPSILON);
      }
   }
}
