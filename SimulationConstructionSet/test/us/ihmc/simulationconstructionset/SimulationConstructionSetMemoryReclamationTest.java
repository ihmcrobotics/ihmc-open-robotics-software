package us.ihmc.simulationconstructionset;

import static org.junit.Assert.assertTrue;

import java.awt.Frame;
import java.io.File;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;
import us.ihmc.tools.MemoryTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.UI})
public class SimulationConstructionSetMemoryReclamationTest
{
   private static final boolean DEBUG = true;

	@ContinuousIntegrationTest(estimatedDuration = 10.3)
	@Test(timeout = 51000)
   public void testMemoryReclamationForSCSWithoutARobot()
   {
      boolean useRobot = false;
      int numberOfTests = 3;
      boolean createVideo = false;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createVideo);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      checkForLingeringFrames();
      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }

	@ContinuousIntegrationTest(estimatedDuration = 6.4)
	@Test(timeout = 32000)
   public void testMemoryReclamationForSCSWithARobot()
   {
      boolean useRobot = true;
      int numberOfTests = 1;
      boolean createVideo = false;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createVideo);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      checkForLingeringFrames();
      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }

   // TODO https://jira.ihmc.us/browse/DRC-2208
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testMemoryReclamationForSCSWithARobotAndVideo()
   {
      boolean useRobot = true;
      int numberOfTests = 10;
      boolean createVideo = true;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createVideo);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      checkForLingeringFrames();
      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 50);
   }

   private int testOneAndReturnUsedMemoryMB(boolean useARobot, int numberOfTests, boolean createVideo)
   {
      boolean garbageCollect = true;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("testOneAndReturnUsedMemoryMB start:", DEBUG, garbageCollect );

      for (int i = 0; i < numberOfTests; i++)
      {
         SimulationConstructionSet scs = createAndStartSimulationConstructionSet(useARobot);
         scs.simulate(2.0);
         
         sleep(2000);
         if (createVideo)
         {
            scs.gotoInPointNow();

            String videoFilename = "testOneAndReturnUsedMemoryMB.mp4";
            File file = new File(videoFilename);
            if (file.exists())
               file.delete();
            
            File videoFile = scs.createVideo(videoFilename);
            videoFile.delete();
            
            printIfDebug("Got past video creation...maybe");
         }
         scs.closeAndDispose();
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("testOneAndReturnUsedMemoryMB final: ", DEBUG, garbageCollect);
      }
      System.gc();

      printIfDebug("Created and disposed of " + numberOfTests + " SCSs. Should be garbage collected now...");
      sleep(2000);
      int usedMemoryMB = MemoryTools.getCurrentMemoryUsageInMB();
      printIfDebug("Used Memory = " + usedMemoryMB + " MB");
      
      return usedMemoryMB;
   }
   
   private void checkForLingeringFrames()
   {
      Frame[] frames = Frame.getFrames();
      if (frames != null)
      {
         printIfDebug("Number of Frames is still " + frames.length);
         for (Frame frame : frames)
         {
            printIfDebug("Frame " + frame.getTitle() + ": " + frame);
         }
      }
      frames = null;
   }

   
   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
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
         
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setDataBufferSize(5000);
         scs = new SimulationConstructionSet(robot, parameters);
      }
      else
      {
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setCreateGUI(true);
         parameters.setDataBufferSize(5000);
         scs = new SimulationConstructionSet(parameters);
      }
      
      scs.setDT(0.0001, 100);

      Thread thread = new Thread(scs);
      thread.start();

      while (useARobot &&!scs.isSimulationThreadRunning())
      {
         sleep(100);
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
