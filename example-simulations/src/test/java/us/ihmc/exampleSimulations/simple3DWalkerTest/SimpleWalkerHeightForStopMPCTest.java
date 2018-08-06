package us.ihmc.exampleSimulations.simple3DWalkerTest;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.exampleSimulations.simple3DWalker.SimpleWalkerHeightStopMPC;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimpleWalkerHeightForStopMPCTest
{
   YoVariableRegistry registry = new YoVariableRegistry("registry");
   double umax=1000;
   double zmax=1.1;
   double zf=1.0;
   double x;

   @Test
   public void testMaxHeightMaxU()
   {
      SimpleWalkerHeightStopMPC heightMPC = new SimpleWalkerHeightStopMPC(zmax,zf,umax,registry);

      for (int i=0;i<10;i++)
      {
         x = -0.3193 + i*0.02;
         double uinit = heightMPC.getLegForceInputU(x,1.0,1.0,0.0);
         Assert.assertTrue("U exceeds defined max", uinit <= umax);
         double zMaxProg = heightMPC.getZmaxComputed();
         Assert.assertTrue("Max height of polynomial exceeds requested", zMaxProg <= zmax);
        // double zMaxPol =
         PrintTools.info("The algorithm took "+ heightMPC.getIters() + " iterations and "+ heightMPC.getComputationTimems()+" ms and dxf= "+ heightMPC.getDxf()+" polcst "
                           + heightMPC.getC()[0]+ " " + heightMPC.getC()[1]+ " "+ heightMPC.getC()[2]+ " "+ heightMPC.getC()[3]+ " ");
      }
   }
}
