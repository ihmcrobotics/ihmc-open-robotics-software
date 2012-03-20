package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertTrue;

import java.awt.Frame;

import org.junit.Test;

public class SimulationConstructionSetMemoryReclamationTest
{
   @Test
   public void testMemoryReclamationForSCSWithARobot()
   {
      int usedMemoryMBAtStart = printMemoryUsageAndReturnUsedMemoryInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(true, 1);
      
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
   
   
   @Test
   public void testMemoryReclamationForSCSWithoutARobot()
   {
      int usedMemoryMBAtStart = printMemoryUsageAndReturnUsedMemoryInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(false, 2);
      
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;
      
      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }
   
//   @Test
//   public void sleepForever()
//   {
//      sleep(3000000);
//   }

   private int testOneAndReturnUsedMemoryMB(boolean useARobot, int numberOfTests)
   {
      printMemoryUsageAndReturnUsedMemoryInMB();

      for (int i = 0; i < numberOfTests; i++)
      {
         SimulationConstructionSet scs = createAndStartSimulationConstructionSet(useARobot);

         sleep(2000);

         scs.closeAndDispose();

         printMemoryUsageAndReturnUsedMemoryInMB();

      }

      System.gc();

      System.out.println("Created and disposed of " + numberOfTests + " SCSs. Should be garbage collected now...");
      sleep(2000);
      int usedMemoryMB = printMemoryUsageAndReturnUsedMemoryInMB();
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
      
      while(useARobot && !scs.isSimulationThreadUpAndRunning())
      {
         sleep(100);
      }
      
      return scs;
   }

   private int printMemoryUsageAndReturnUsedMemoryInMB()
   {
      Runtime runtime = Runtime.getRuntime();

      System.gc();
      sleep(100);
      
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
