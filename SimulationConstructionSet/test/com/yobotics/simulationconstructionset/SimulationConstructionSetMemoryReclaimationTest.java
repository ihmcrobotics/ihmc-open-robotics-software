package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertTrue;

import java.awt.Frame;

import javax.swing.JFrame;

import org.junit.Ignore;
import org.junit.Test;

public class SimulationConstructionSetMemoryReclaimationTest
{
   @Ignore
   public void testMemoryReclaimationForSCSWithoutARobot()
   {
      int usedMemoryMB = testOneAndReturnUsedMemoryMB(false, 5);
      
      assertTrue(usedMemoryMB < 50);

   }

   @Ignore
   public void testMemoryReclaimationForSCSWithARobot()
   {
      int usedMemoryMB = testOneAndReturnUsedMemoryMB(true, 1);
      
      
      Frame[] frames = Frame.getFrames();
      if (frames != null)
      {
         System.out.println("Number of Frames is still " + frames.length);
         for (Frame frame : frames)
         {
            System.out.println(frame.getName() + ": " + frame);
            
            JFrame jFrame = (JFrame) frame;
            jFrame.getRootPane().removeAll();
            jFrame.setVisible(false);
            jFrame.dispose();
         }
      }
      
      frames = null;
      
//      assertTrue(usedMemoryMB < 50);
   }
   
   @Ignore
   public void sleepForever()
   {
      sleep(3000000);
   }

   private int testOneAndReturnUsedMemoryMB(boolean useARobot, int numberOfTests)
   {
      Runtime runtime = Runtime.getRuntime();
      printMemoryUsageAndReturnUsedMemoryInMB(runtime);

      for (int i = 0; i < numberOfTests; i++)
      {
         SimulationConstructionSet scs = createAndStartSimulationConstructionSet(useARobot);

         sleep(2000);

         scs.closeAndDispose();

         System.gc();
         printMemoryUsageAndReturnUsedMemoryInMB(runtime);

      }

      System.gc();

      System.out.println("Created and disposed of " + numberOfTests + " SCSs. Should be garbage collected now...");
      sleep(2000);
      int usedMemoryMB = printMemoryUsageAndReturnUsedMemoryInMB(runtime);
      return usedMemoryMB;
   }

   private SimulationConstructionSet createAndStartSimulationConstructionSet(boolean useARobot)
   {
      SimulationConstructionSet scs;

      if (useARobot)
      {
         Robot robot = new Robot("TestRobot");
         
         YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
         
         for (int i=0; i<5000; i++)
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
      
      while(!scs.isSimulationThreadUpAndRunning())
      {
         sleep(100);
      }
      
      return scs;
   }

   private int printMemoryUsageAndReturnUsedMemoryInMB(Runtime runtime)
   {
      long freeMemory = runtime.freeMemory();
      long totalMemory = runtime.totalMemory();
      long usedMemory = totalMemory - freeMemory;

      int usedMemoryMB = (int) (usedMemory / 1000000);
      System.out.println("freeMemory = " + freeMemory / 1000000 + "MB, totalMemory = " + totalMemory / 1000000 + "MB, usedMemory = " + usedMemoryMB + "MB");

      return usedMemoryMB;
   }

   private void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      } catch (InterruptedException e)
      {
      }

   }

}
