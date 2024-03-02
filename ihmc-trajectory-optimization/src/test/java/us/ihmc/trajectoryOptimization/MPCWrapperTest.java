package us.ihmc.trajectoryOptimization;

import org.junit.jupiter.api.Test;
import us.ihmc.mpc.mpcWrapper.MPCWrapper;
import us.ihmc.mpc.mpcWrapper.presets.MPCWrapperNativeLibrary;

public class MPCWrapperTest
{
   @Test
   public void testNativeMPCWrapperLibrary()
   {
      MPCWrapperNativeLibrary.load();

      MPCWrapper.MPCExternal mpc = new MPCWrapper.MPCExternal();

      float[] poseInitial = new float[] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      float[] odometry = new float[] {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
   }
}
