package us.ihmc.simulationconstructionset.util.simulationTesting;

import static org.junit.Assert.*;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.StateFileComparer;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;

public class SimulationTestTools
{
   public static void compare(List<File> stateFiles, SimulationConstructionSet scs, List<String> namesToSkipOver)
   {
      double maxPercentDiff = Double.MIN_VALUE;
      ArrayList<VariableDifference> variableDifferences = StateFileComparer.percentualCompareStateFiles(stateFiles.get(0), stateFiles.get(1), maxPercentDiff,
                                                             namesToSkipOver);

      deleteStateFiles(stateFiles);
      assertChangedVarsEmpty(scs, variableDifferences);
      scs = null;
   }

   public static SimulationConstructionSet runNumberOfTicks(SimulationConstructionSet scs, int nTicks)
   {
      scs.simulate(nTicks);
      waitForSimulationToFinish(scs);
//      System.out.println("Simulation run for " + nTicks + " ticks");

      return scs;
   }

   public static ArrayList<File> createStateFiles(int nStateFiles)
   {
      ArrayList<File> stateFiles = new ArrayList<File>();
      for (int i = 0; i < nStateFiles; i++)
      {
         stateFiles.add(new File("stateFile" + i));
      }

      return stateFiles;
   }

   public static void compareStateFiles(ArrayList<File> stateFiles, SimulationConstructionSet scs)
   {
      double maxPercentDiff = Double.MIN_VALUE;
      ArrayList<VariableDifference> variableDifferences = StateFileComparer.percentualCompareStateFiles(stateFiles.get(0), stateFiles.get(1), maxPercentDiff,
                                                             null);
      deleteStateFiles(stateFiles);
      assertChangedVarsEmpty(scs, variableDifferences);
   }

   // Private Methods

   private static void waitForSimulationToFinish(SimulationConstructionSet scs)
   {
      while (scs.isSimulating())
      {
         try
         {
            Thread.sleep(10);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }

   private static void deleteStateFiles(List<File> stateFiles)
   {
      for (File stateFile : stateFiles)
      {
         stateFile.delete();
      }
   }

   public static void assertChangedVarsEmpty(SimulationConstructionSet scs, List<VariableDifference> variableDifferences)
   {
//    if (variableDifferences.containsVariable(scs.getVariable("t")))
//    {
//       throw new RuntimeException("times not equal.");
//    }

      String message = createFailMessage(scs.getAllVariables(), variableDifferences);

      scs = null;
      assertEquals(message, true, variableDifferences.isEmpty());
   }

   private static String createFailMessage(List<YoVariable<?>> allVariables, List<VariableDifference> variableDifferences)
   {
      String numberOfVarsChangedString = variableDifferences.size() + " out of " + allVariables.size() + " YoVariables changed.\n";
      String changedVarsString = "Changed vars:\n" + VariableDifference.allVariableDifferencesToString(variableDifferences);
      String message = numberOfVarsChangedString + changedVarsString;

      return message;
   }

   /**
    * Simple rewindability test.
    * run, write state file, rewind, run, write state file.
    * @throws SimulationExceededMaximumTimeException
    */
   public static void testRewindabilitySimple(SimulationConstructionSet scs, double simulationTime, ArrayList<String> namesToSkipOver)
           throws SimulationExceededMaximumTimeException
   {
      // create state files
      int nStateFiles = 2;
      ArrayList<File> stateFiles = createStateFiles(nStateFiles);

//      System.out.println("Simulation created");

      // run, write state file, rewind, repeat
      for (int i = 0; i < nStateFiles; i++)
      {
         scs.simulate(simulationTime);
         BlockingSimulationRunner.waitForSimulationToFinish(scs, 120.0, true);
         scs.writeState(stateFiles.get(i));
//         System.out.println("Run " + i + " done.");
         scs.gotoInPointNow();
         assertEquals(scs.getIndex(), 0);
      }

      // compare
      compare(stateFiles, scs, namesToSkipOver);
   }

   public static void testRewindabilityNumberOfTicks(SimulationConstructionSet scs, int nTicks, List<String> namesToSkipOver)
           throws SimulationExceededMaximumTimeException
   {
      // create state files
      int nStateFiles = 2;
      ArrayList<File> stateFiles = createStateFiles(nStateFiles);

//      System.out.println("Simulation created");

      // run, write state file, rewind, repeat
      for (int i = 0; i < nStateFiles; i++)
      {
         scs.simulate(nTicks);
         BlockingSimulationRunner.waitForSimulationToFinish(scs, 120.0, true);
         scs.writeState(stateFiles.get(i));
//         System.out.println("Run " + i + " done.");
         scs.gotoInPointNow();
         assertEquals(scs.getIndex(), 0);
      }

      compare(stateFiles, scs, namesToSkipOver);
   }

   public static void testRandom(ArrayList<SimulationConstructionSet> scss, Random random) throws SimulationExceededMaximumTimeException
   {
      double epsilon = 1e-12;
      double maximumClockRunTimeInSeconds = 120.0;
      int nIterations = 10;
      double maxSimTime = 0.25;
      double maxRewindTime = 0.5;

      AllYoVariablesSimulationComparer comparer = new AllYoVariablesSimulationComparer(epsilon);

      comparer.addException("DurationMilli");
      comparer.addException("TimeNano");

      /*
       * 0: simulate a random amount of time, rewind, simulate 1 tick
       * 1: simulate 1 tick
       */
      for (int i = 0; i < nIterations; i++)
      {
//         System.out.println("Starting Run " + i + ". Current index: " + scss.get(0).getIndex());

         double simTime = random.nextDouble() * maxSimTime;

         for (int y = 0; y < scss.size(); y++)
         {
            scss.get(y).simulate(simTime);
         }

         for (int y = 0; y < scss.size(); y++)
         {
            BlockingSimulationRunner.waitForSimulationToFinish(scss.get(y), maximumClockRunTimeInSeconds, true);
         }

         double rewindTime = random.nextDouble() * maxRewindTime;

         scss.get(0).setInPoint();
         scss.get(0).simulate(rewindTime);
         scss.get(1).simulate(1);

         for (int y = 0; y < scss.size(); y++)
         {
            BlockingSimulationRunner.waitForSimulationToFinish(scss.get(y), maximumClockRunTimeInSeconds, true);
         }

         scss.get(0).gotoInPointNow();
         scss.get(0).simulate(1);
         BlockingSimulationRunner.waitForSimulationToFinish(scss.get(0), maximumClockRunTimeInSeconds, true);
         assertEquals("indices not the same", scss.get(0).getIndex(), scss.get(1).getIndex());
         assertEquals("times not the same", scss.get(0).getRobots()[0].getTime(), scss.get(1).getRobots()[0].getTime(), epsilon);
         boolean result = comparer.compare(scss.get(0), scss.get(1));

         if (!result)
         {
            fail(comparer.toString());
         }
      }
   }
   
   public static void testInitialValuesStoredCorrectly(SimulationConstructionSet scs, List<String> namesToSkipOver, double maximumClockRunTimeInSeconds) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      // TODO: Not sure why the qdd are different on the very first tick, but we should probably fix this.
      namesToSkipOver.add("qdd_");
      
      int nStateFiles = 2;
      ArrayList<File> stateFiles = SimulationTestTools.createStateFiles(nStateFiles);

      // write state file, run, rewind, write state file
      scs.writeState(stateFiles.get(0));

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, maximumClockRunTimeInSeconds, true);
      blockingSimulationRunner.simulateNTicksAndBlock(1);
      scs.stepBackwardNow();
      scs.writeState(stateFiles.get(1));

      // compare
      SimulationTestTools.compare(stateFiles, scs, namesToSkipOver);
   }
}
