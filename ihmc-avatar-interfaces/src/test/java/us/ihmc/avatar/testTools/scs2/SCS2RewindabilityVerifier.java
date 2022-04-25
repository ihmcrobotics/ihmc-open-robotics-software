package us.ihmc.avatar.testTools.scs2;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariablesThatShouldMatchList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2RewindabilityVerifier
{
   private static final boolean DEBUG = false;

   private final SCS2AvatarTestingSimulation[] simulations;
   private final List<String> exceptions;

   public SCS2RewindabilityVerifier(SCS2AvatarTestingSimulation simulationOne, SCS2AvatarTestingSimulation simulationTwo, List<String> exceptions)
   {
      simulations = new SCS2AvatarTestingSimulation[] {simulationOne, simulationTwo};
      this.exceptions = exceptions;
   }

   public SCS2RewindabilityVerifier(SCS2AvatarTestingSimulation[] simulations, List<String> exceptions)
   {
      this.exceptions = exceptions;

      if (simulations.length != 2)
      {
         throw new RuntimeException("Need exactly 2 simulations in order to do rewindability verifier");
      }

      this.simulations = simulations;
   }

   public List<VariableDifference> verifySimulationsAreSameToStart()
   {
      double time = simulations[0].getSimulationTime();

      YoRegistry registry0 = simulations[0].getRootRegistry();
      YoRegistry registry1 = simulations[1].getRootRegistry();

      VariablesThatShouldMatchList variablesThatShouldMatchList = new VariablesThatShouldMatchList(registry0, registry1, exceptions);

      boolean checkForPercentDifference = false;
      double maxDifferenceAllowed = Double.MIN_VALUE;
      ArrayList<VariableDifference> variableDifferences = new ArrayList<>();

      variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, time, maxDifferenceAllowed, checkForPercentDifference);

      return variableDifferences;
   }

   public List<VariableDifference> checkRewindabilityWithSimpleMethod(int numTicksToTest, double maxDifferenceAllowed)
   {
      int numTicksToStartComparingAt = 1;
      return checkRewindabilityWithSimpleMethod(numTicksToStartComparingAt, numTicksToTest, maxDifferenceAllowed);
   }

   public List<VariableDifference> checkRewindabilityWithSimpleMethod(int numTicksToStartComparingAt, int numTicksToTest, double maxDifferenceAllowed)
   {
      List<VariableDifference> variableDifferencesToReturn = new ArrayList<>();
      checkRewindabilityWithSimpleMethod(numTicksToStartComparingAt, numTicksToTest, maxDifferenceAllowed, variableDifferencesToReturn);

      return variableDifferencesToReturn;
   }

   /**
    * This is a simple rewindability checker. For the first simulation it ticks it ahead one tick at a
    * time. For the second simulation it ticks it ahead a tick, backs up a tick, and ticks ahead
    * another tick. Then it compares the two simulations. It repeats this for the indicated number of
    * times. Any differences are put in the returned ArrayList.
    */
   public int checkRewindabilityWithSimpleMethod(int numTicksToStartComparingAt,
                                                 int numTicksToTest,
                                                 double maxDifferenceAllowed,
                                                 List<VariableDifference> variableDifferencesToPack)
   {
      int numTicksToSimulateAhead = 1;
      return checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt,
                                                  numTicksToTest,
                                                  numTicksToSimulateAhead,
                                                  maxDifferenceAllowed,
                                                  variableDifferencesToPack);
   }

   public List<VariableDifference> checkRewindabilityWithRigorousMethod(int numTicksToStartComparingAt,
                                                                        int numTicksToTest,
                                                                        int numTicksToSimulateAhead,
                                                                        double maxDifferenceAllowed)
   {
      List<VariableDifference> variableDifferencesToReturn = new ArrayList<>();
      checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt,
                                           numTicksToTest,
                                           numTicksToSimulateAhead,
                                           maxDifferenceAllowed,
                                           variableDifferencesToReturn);
      return variableDifferencesToReturn;
   }

   /**
    * This is a more rigorous rewindability checker. For the first simulation it ticks it ahead one
    * tick at a time. For the second simulation it ticks it ahead numTicksToSimulateAhead ticks,
    * rewinds it to where it had started, and ticks ahead another tick. Then it compares the two
    * simulations. It repeats this for the indicated number of numTicksToTest. Any differences are put
    * in the returned ArrayList.
    */
   public int checkRewindabilityWithRigorousMethod(int numTicksToStartComparingAt,
                                                   int numTicksToTest,
                                                   int numTicksToSimulateAhead,
                                                   double maxDifferenceAllowed,
                                                   List<VariableDifference> variableDifferencesToPack)
   {
      YoRegistry registry0 = simulations[0].getRootRegistry();
      YoRegistry registry1 = simulations[1].getRootRegistry();

      variableDifferencesToPack.clear();
      VariablesThatShouldMatchList variablesThatShouldMatchList = new VariablesThatShouldMatchList(registry0, registry1, exceptions);

      simulateForNumberOfTicks(simulations[0], numTicksToStartComparingAt);
      simulateForNumberOfTicks(simulations[1], numTicksToStartComparingAt);

      for (int tickIndex = 0; tickIndex < numTicksToTest; tickIndex++)
      {
         boolean passesTest = verifyMatch(variablesThatShouldMatchList, variableDifferencesToPack, maxDifferenceAllowed);
         if (!passesTest)
         {
            System.err.println("Was not rewindable. Failed on tick " + tickIndex);
            return tickIndex;
         }

         if (DEBUG)
            System.out.println("SimulationRewindabilityVerifier: Passed sims match test on loop " + tickIndex + " of " + numTicksToTest);

         simulations[0].simulateOneBufferRecordPeriodNow();

         for (int i = 0; i < numTicksToSimulateAhead; i++)
         {
            simulations[1].simulateOneBufferRecordPeriodNow();
         }
         for (int i = 0; i < numTicksToSimulateAhead; i++)
         {
            simulations[1].stepBufferIndexBackward();
         }

         simulations[1].simulateOneBufferRecordPeriodNow();
      }

      return numTicksToTest;
   }

   /**
    * This is the same rewindability checker as above, but also records each individual YoVariable
    * change using YoVariableListeners in the SimulationRewindabilityHelper. Once a difference is found
    * between the simulations, it prints the stack trace of the first changes that were made that
    * resulted in variables being different.
    *
    * @param numTicksToStartComparingAt
    * @param numTicksToTest
    * @param maxDifferenceAllowed
    * @return
    */
   public void checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(int numTicksToStartComparingAt,
                                                                                      int numTicksToTest,
                                                                                      double maxDifferenceAllowed,
                                                                                      List<VariableDifference> variableDifferencesToPack)
   {
      variableDifferencesToPack.clear();

      simulateForNumberOfTicks(simulations[0], numTicksToStartComparingAt);
      simulateForNumberOfTicks(simulations[1], numTicksToStartComparingAt);

      SCS2RewindabilityVerifierWithStackTracing helper = new SCS2RewindabilityVerifierWithStackTracing(simulations[0], simulations[1], exceptions);

      for (int tickIndex = 0; tickIndex < numTicksToTest; tickIndex++)
      {
         boolean areTheVariableChangesDifferent = helper.areTheVariableChangesDifferent();
         if (areTheVariableChangesDifferent)
         {
            helper.printOutStackTracesOfFirstChangedVariable();
            return; //DEBUG REWINDABILITY
         }

         if (DEBUG)
            System.out.println("SimulationRewindabilityVerifier: Passed sims match test on loop " + tickIndex + " of " + numTicksToTest);

         helper.clearChangesForSimulations();

         helper.setRecordDifferencesForSimOne(true);
         helper.setRecordDifferencesForSimTwo(false);
         simulations[0].simulateOneBufferRecordPeriodNow();
         helper.setRecordDifferencesForSimOne(false);

         int numberOfStepsForward = 1;
         for (int i = 0; i < numberOfStepsForward; i++)
         {
            simulations[1].simulateOneBufferRecordPeriodNow();
         }
         for (int i = 0; i < numberOfStepsForward; i++)
         {
            simulations[1].stepBufferIndexBackward();
         }

         helper.setRecordDifferencesForSimTwo(true);
         simulations[1].simulateOneBufferRecordPeriodNow();
      }
   }

   public void simulateForNumberOfTicks(int numberOfTicks)
   {
      simulateForNumberOfTicks(simulations[0], numberOfTicks);
      simulateForNumberOfTicks(simulations[1], numberOfTicks);
   }

   private static void simulateForNumberOfTicks(SCS2AvatarTestingSimulation simulationTestHelper, int numberOfTicks)
   {
      for (int i = 0; i < numberOfTicks; i++)
      {
         simulationTestHelper.simulateOneBufferRecordPeriodNow();
      }
   }

   private boolean verifyMatch(VariablesThatShouldMatchList variablesThatShouldMatchList,
                               List<VariableDifference> variableDifferencesToPack,
                               double maxDifferenceAllowed)
   {
      double time = simulations[0].getSimulationTime();

      boolean checkForPercentDifference = false;

      return variablesThatShouldMatchList.doVariableValuesMatch(variableDifferencesToPack, time, maxDifferenceAllowed, checkForPercentDifference);
   }

}
