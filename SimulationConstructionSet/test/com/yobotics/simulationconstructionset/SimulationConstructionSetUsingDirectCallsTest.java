package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertEquals;

import java.awt.AWTException;

import org.junit.Test;

import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.examples.FallingBrickRobot;

public class SimulationConstructionSetUsingDirectCallsTest
{

   @Test
   public void testSimulationConstructionSetUsingDirectCalls() throws AWTException
   {
      FallingBrickRobot simpleRobot = new FallingBrickRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();
      
      double simulateTime = 1.0;
      
      double startTime = scs.getTime();
      scs.simulate(simulateTime);
      while(scs.isSimulating())
      {
         ThreadTools.sleep(100L);
      }
      
      double endTime = scs.getTime();

      assertEquals(simulateTime, endTime-startTime, 1e-7);

//      ThreadTools.sleepForever();
      
      scs.closeAndDispose();
      scs = null;
   }
 

}
