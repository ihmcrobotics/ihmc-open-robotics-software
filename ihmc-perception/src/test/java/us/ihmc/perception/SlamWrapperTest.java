package us.ihmc.perception;

import org.junit.jupiter.api.Test;
import us.ihmc.perception.slamWrapper.SlamWrapper;
import us.ihmc.perception.slamWrapper.SlamWrapperNativeLibrary;

public class SlamWrapperTest
{
   @Test
   public void testNativeSlamWrapperLibrary()
   {
      SlamWrapperNativeLibrary.load();

      SlamWrapper.FactorGraphExternal factorGraphExternal = new SlamWrapper.FactorGraphExternal();

      float[] poseInitial = new float[] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      float[] odometry = new float[] {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};

      factorGraphExternal.addPriorPoseFactor(1, poseInitial);
      factorGraphExternal.addOdometryFactor(2, odometry);

      factorGraphExternal.setPoseInitialValue(1, poseInitial);
      factorGraphExternal.setPoseInitialValue(2, odometry);

      factorGraphExternal.optimize();

      factorGraphExternal.printResults();
   }
}
