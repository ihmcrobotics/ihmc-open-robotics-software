package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertTrue;

import java.awt.Frame;
import java.io.File;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.DateTools;
import us.ihmc.utilities.MemoryTools;

import com.yobotics.simulationconstructionset.examples.FallingBrickRobot;

public class SimulationConstructionSetMemoryReclamationTest
{
   @Test
   public void testMemoryReclamationForSCSWithoutARobot()
   {
      boolean useRobot = false;
      int numberOfTests = 3;
      boolean createMovie = false;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createMovie);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }

   @Test
   public void testMemoryReclamationForSCSWithARobot()
   {
      boolean useRobot = true;
      int numberOfTests = 1;
      boolean createMovie = false;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createMovie);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      Frame[] frames = Frame.getFrames();
      if (frames != null)
      {
         System.out.println("Number of Frames is still " + frames.length);
         for (Frame frame : frames)
         {
            System.out.println("Frame " + frame.getTitle() + ": " + frame);
         }
      }
      frames = null;

      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }

   @Ignore
   @Test
   public void testMemoryReclamationForSCSWithARobotAndMovie()
   {
      boolean useRobot = true;
      int numberOfTests = 1;
      boolean createMovie = true;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createMovie);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      Frame[] frames = Frame.getFrames();
      if (frames != null)
      {
         System.out.println("Number of Frames is still " + frames.length);
         for (Frame frame : frames)
         {
            System.out.println("Frame " + frame.getTitle() + ": " + frame);
         }
      }
      frames = null;

//      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }

   private int testOneAndReturnUsedMemoryMB(boolean useARobot, int numberOfTests, boolean createMovie)
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("testOneAndReturnUsedMemoryMB start:");

      for (int i = 0; i < numberOfTests; i++)
      {
         SimulationConstructionSet scs = createAndStartSimulationConstructionSet(useARobot);
         sleep(2000);
         if(createMovie)
         {
            createMovieAndDataWithDateTimeClassMethod("/SimulationConstructionSet/resources/", "BrettRobot2000", scs, 2);
            System.err.println("Movie creation successful");
         }
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
         sleep(100);
      }
      return scs;
   }
   
   private static File[] createMovieAndDataWithDateTimeClassMethod(String rootDirectory, String simplifiedRobotModelName, SimulationConstructionSet scs, int stackDepthForRelevantCallingMethod)
   {
      String dateString = DateTools.getDateString();
      String directoryName = rootDirectory + dateString + "/";

      File directory = new File(directoryName);
      System.err.println(directory.getName());
      if (!directory.exists())
      {
         boolean result = directory.mkdir();
         System.err.println("DIRECTORY CREATED: " + result);
      }

//      String timeString = DateTools.getTimeString();
//      String filenameStart = dateString + "_" + timeString;
//      if (!simplifiedRobotModelName.equals(""))
//      {
//         filenameStart += "_" + simplifiedRobotModelName;
//      }
//
//      String movieFilename = filenameStart + ".mp4";
//
//      File movieFile;
//      movieFile = scs.createMovie(directoryName + movieFilename);
//
//      String dataFilename = directoryName + filenameStart + ".data.gz";
//
//      File dataFile = new File(dataFilename);
//      try
//      {
//         scs.writeData(dataFile);
//      } catch (Exception e)
//      {
//         System.err.println("Error in writing data file in BambooTools.createMovieAndDataWithDateTimeClassMethod()");
//         e.printStackTrace();
//      }
//
//      scs.gotoOutPointNow();
      return new File[]{directory};

//      return new File[]{directory, movieFile, dataFile};
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