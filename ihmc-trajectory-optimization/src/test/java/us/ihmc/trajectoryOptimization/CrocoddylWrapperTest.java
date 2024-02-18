package us.ihmc.trajectoryOptimization;

import org.junit.jupiter.api.Test;
import us.ihmc.crocoddyl.crocoddylWrapper.CrocoddylWrapperNativeLibrary;

public class SlamWrapperTest
{
   @Test
   public void testNativeSlamWrapperLibrary()
   {
      CrocoddylWrapperNativeLibrary.load();

      CrocoddylExternal factorGraphExternal = new CrocoddylExternal();
   }
}

