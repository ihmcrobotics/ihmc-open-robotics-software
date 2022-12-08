package us.ihmc.perception.slamWrapper.test;

import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.slamWrapper.SlamWrapper;

public class SlamWrapperTest
{
   public static void main(String[] args)
   {
      BytedecoTools.loadSlamWrapper();

      SlamWrapper.FactorGraphExternal factorGraphExternal = new SlamWrapper.FactorGraphExternal();

//      factorGraphExternal.helloWorldTest();

      float[] poseInitial = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      float[] odometry = new float[]{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};

      factorGraphExternal.addPriorPoseFactor(1, poseInitial);
      factorGraphExternal.addOdometryFactor(odometry, 2);

      factorGraphExternal.setPoseInitialValue(1, poseInitial);
      factorGraphExternal.setPoseInitialValue(2, odometry);

      factorGraphExternal.optimize();

      factorGraphExternal.printResults();
   }
}
