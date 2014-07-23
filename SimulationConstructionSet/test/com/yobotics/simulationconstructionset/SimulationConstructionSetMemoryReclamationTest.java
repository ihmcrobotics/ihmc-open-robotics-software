package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertTrue;
import java.awt.Frame;
import org.junit.Test;
import com.yobotics.exampleSimulations.fallingBrick.FallingBrickRobot;
import us.ihmc.utilities.MemoryTools;

public class SimulationConstructionSetMemoryReclamationTest
{
   @Test
   public void testMemoryReclamationForSCSWithoutARobot()
   {
      boolean useRobot = false;
      int numberOfTests = 3;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }
   
   @Test
   public void testMemoryReclamationForSCSWithARobot()
   {
      boolean useRobot = true;
      int numberOfTests = 1;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      Frame[] frames = Frame.getFrames();
      if (frames != null)
      {
         System.out.println("Number of Frames is still " + frames.length);
         for (Frame frame : frames)
         {
            System.out.println(frame.getTitle() + ": " + frame);

            //            frame.setVisible(false);
            //            frame.dispose();
         }
      }
      frames = null;

      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }

   private int testOneAndReturnUsedMemoryMB(boolean useARobot, int numberOfTests)
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("testOneAndReturnUsedMemoryMB start:");

      for (int i = 0; i < numberOfTests; i++)
      {
         SimulationConstructionSet scs = createAndStartSimulationConstructionSet(useARobot);
         sleep(200000); //2000
         scs.closeAndDispose();
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("testOneAndReturnUsedMemoryMB final: ");
      }
      System.gc();

      System.out.println("Created and disposed of " + numberOfTests + " SCSs. Should be garbage collected now...");
      sleep(2000);
      int usedMemoryMB = MemoryTools.getCurrentMemoryUsageInMB();
      return usedMemoryMB;
   }

   private SimulationConstructionSet createAndStartSimulationConstructionSet(boolean useARobot)
   {
      SimulationConstructionSet scs;

      if (useARobot)
      {
         FallingBrickRobot robot = new FallingBrickRobot();
         YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");

         for (int i = 0; i < 5000; i++)
         {
            new DoubleYoVariable("variable" + i, registry);
         }

         robot.addYoVariableRegistry(registry);
         scs = new SimulationConstructionSet(robot, 5000);
      }
      else
      {
         scs = new SimulationConstructionSet(true, 5000);
      }

      Thread thread = new Thread(scs);
      thread.start();

      while (useARobot && !scs.isSimulationThreadUpAndRunning())
      {
         sleep(10000); //100
      }
      return scs;
   }

   private void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      }
      catch (InterruptedException e)
      {
      }
   }
}