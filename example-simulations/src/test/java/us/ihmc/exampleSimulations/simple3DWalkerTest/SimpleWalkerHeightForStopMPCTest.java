package us.ihmc.exampleSimulations.simple3DWalkerTest;

import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.PrintTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.exampleSimulations.simple3DWalker.SimpleWalkerHeightStopMPC;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimpleWalkerHeightForStopMPCTest
{
   YoRegistry registry = new YoRegistry("registry");
   double umax = 1000;
   double zmax = 1.1;
   double zf = 1.0;
   double x;

   SimpleWalkerHeightStopMPC heightMPC = new SimpleWalkerHeightStopMPC(zmax, zf, umax, registry);

   @Test
   public void testMaxHeightMaxUOutLoop()
   {
      for (int i = 0; i < 10; i++)
      {
         x = -0.3193 + i * 0.02;
         heightMPC.computeInvOutLoop(x, 1.0, 1.0, 0.0);
         double uinit = heightMPC.getU();
         Assert.assertTrue("U exceeds defined max", uinit <= umax);
         double zMaxProg = heightMPC.getZmaxComputed();
         Assert.assertTrue("Max height of polynomial exceeds requested", zMaxProg <= zmax);
         // double zMaxPol =
         if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         {
            PrintTools.info("The algorithm took " + heightMPC.getIters() + " iterations and " + heightMPC.getComputationTimems() + " ns and dxf= " + heightMPC
                  .getDxf() + " polcst " + heightMPC.getC()[0] + " " + heightMPC.getC()[1] + " " + heightMPC.getC()[2] + " " + heightMPC.getC()[3] + " ");
         }
      }
   }

   @Test
   public void testMaxHeightMaxUInLoop()
   {
      for (int i = 0; i < 10; i++)
      {
         x = -0.3193 + i * 0.02;
         heightMPC.computeInvInLoop(x, 1.0, 1.0, 0.0);
         double uinit = heightMPC.getU();
         Assert.assertTrue("U exceeds defined max", uinit <= umax);
         double zMaxProg = heightMPC.getZmaxComputed();
         Assert.assertTrue("Max height of polynomial exceeds requested", zMaxProg <= zmax);
         // double zMaxPol =
         if(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         {
            PrintTools.info("The algorithm took " + heightMPC.getIters() + " iterations and " + heightMPC.getComputationTimems() + " ns and dxf= " + heightMPC.getDxf()
                                  + " polcst " + heightMPC.getC()[0] + " " + heightMPC.getC()[1] + " " + heightMPC.getC()[2] + " " + heightMPC.getC()[3] + " ");
         }
      }
   }
}
