package us.ihmc.simulationconstructionset.util.simulationRunner;

import java.util.ArrayList;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;

public class SimulationRewindabilityVerifier
{
   private static final boolean DEBUG = false;

   private final SimulationConstructionSet[] simulations;
   private final ArrayList<String> exceptions;


   public SimulationRewindabilityVerifier(SimulationConstructionSet simulationOne, SimulationConstructionSet simulationTwo, ArrayList<String> exceptions)
   {
      simulations = new SimulationConstructionSet[] {simulationOne, simulationTwo};
      this.exceptions = exceptions;
   }

   public SimulationRewindabilityVerifier(SimulationConstructionSet[] simulations, ArrayList<String> exceptions)
   {
      this.exceptions = exceptions;

      if (simulations.length != 2)
      {
         throw new RuntimeException("Need exactly 2 simulations in order to do rewindability verifier");
      }

      this.simulations = simulations;
   }


   public ArrayList<VariableDifference> verifySimulationsAreSameToStart()
   {
      double time = simulations[0].getTime();

      YoVariableRegistry registry0 = simulations[0].getRootRegistry();
      YoVariableRegistry registry1 = simulations[1].getRootRegistry();

      VariablesThatShouldMatchList variablesThatShouldMatchList = new VariablesThatShouldMatchList(registry0, registry1, exceptions);

      boolean checkForPercentDifference = false;
      double maxDifferenceAllowed = Double.MIN_VALUE;
      ArrayList<VariableDifference> variableDifferences = new ArrayList<VariableDifference>();

      variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, time, maxDifferenceAllowed, checkForPercentDifference);

      return variableDifferences;
   }


   public ArrayList<VariableDifference> checkRewindabilityWithSimpleMethod(int numTicksToTest, double maxDifferenceAllowed) throws UnreasonableAccelerationException
   { 
      int numTicksToStartComparingAt = 1;
      return checkRewindabilityWithSimpleMethod(numTicksToStartComparingAt, numTicksToTest, maxDifferenceAllowed);
   }

   public ArrayList<VariableDifference> checkRewindabilityWithSimpleMethod(int numTicksToStartComparingAt, int numTicksToTest, double maxDifferenceAllowed) throws UnreasonableAccelerationException
   {
      ArrayList<VariableDifference> variableDifferencesToReturn = new ArrayList<VariableDifference>();
      checkRewindabilityWithSimpleMethod(numTicksToStartComparingAt, numTicksToTest, maxDifferenceAllowed, variableDifferencesToReturn);

      return variableDifferencesToReturn;
   }
         
   /**
    * This is a simple rewindability checker. For the first simulation it ticks it ahead one tick at a time.
    * For the second simulation it ticks it ahead a tick, backs up a tick, and ticks ahead another tick.
    * Then it compares the two simulations. It repeats this for the indicated number of times.
    * Any differences are put in the returned ArrayList.
    */
   public int checkRewindabilityWithSimpleMethod(int numTicksToStartComparingAt, int numTicksToTest, double maxDifferenceAllowed, ArrayList<VariableDifference> variableDifferencesToPack)
           throws UnreasonableAccelerationException
   {
      int numTicksToSimulateAhead = 1;
      return checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt, numTicksToTest, numTicksToSimulateAhead, maxDifferenceAllowed, variableDifferencesToPack);
   }
   
   public ArrayList<VariableDifference> checkRewindabilityWithRigorousMethod(int numTicksToStartComparingAt, int numTicksToTest, int numTicksToSimulateAhead, double maxDifferenceAllowed) throws UnreasonableAccelerationException
   {
      ArrayList<VariableDifference> variableDifferencesToReturn = new ArrayList<VariableDifference>();
      checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt, numTicksToTest, numTicksToSimulateAhead, maxDifferenceAllowed, variableDifferencesToReturn);
      return variableDifferencesToReturn;
   }
   
   /**
    * This is a more rigorous rewindability checker. For the first simulation it ticks it ahead one tick at a time.
    * For the second simulation it ticks it ahead numTicksToSimulateAhead ticks, rewinds it to where it had started,
    * and ticks ahead another tick.
    * Then it compares the two simulations. It repeats this for the indicated number of numTicksToTest.
    * Any differences are put in the returned ArrayList.
    */
   public int checkRewindabilityWithRigorousMethod(int numTicksToStartComparingAt, int numTicksToTest, int numTicksToSimulateAhead, double maxDifferenceAllowed, ArrayList<VariableDifference> variableDifferencesToPack)
           throws UnreasonableAccelerationException
   {
      YoVariableRegistry registry0 = simulations[0].getRootRegistry();
      YoVariableRegistry registry1 = simulations[1].getRootRegistry();

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

         if (DEBUG) System.out.println("SimulationRewindabilityVerifier: Passed sims match test on loop " + tickIndex + " of " + numTicksToTest);

         simulations[0].simulateOneRecordStepNow();
         
         for (int i=0; i< numTicksToSimulateAhead; i++)
         {
            simulations[1].simulateOneRecordStepNow();
         }
         for (int i=0; i< numTicksToSimulateAhead; i++)
         {
            simulations[1].stepBackwardNow();
         }
         
         simulations[1].simulateOneRecordStepNow();
      }

      return numTicksToTest;
   }
   

   /**
    * This is the same rewindability checker as above, but also records each individual YoVariable change using YoVariableListeners 
    * in the SimulationRewindabilityHelper. Once a difference is found between the simulations, it prints the stack trace of the 
    * first changes that were made that resulted in variables being different.
    * 
    * @param numTicksToStartComparingAt
    * @param numTicksToTest
    * @param maxDifferenceAllowed
    * @return 
    * @throws UnreasonableAccelerationException
    */
   public void checkRewindabilityUsingIndividualVariableChangesAndTrackingStackTraces(int numTicksToStartComparingAt,
           int numTicksToTest, double maxDifferenceAllowed, ArrayList<VariableDifference> variableDifferencesToPack)
           throws UnreasonableAccelerationException
   {
      variableDifferencesToPack.clear();

      simulateForNumberOfTicks(simulations[0], numTicksToStartComparingAt);
      simulateForNumberOfTicks(simulations[1], numTicksToStartComparingAt);

      SimulationRewindabilityVerifierWithStackTracing helper = new SimulationRewindabilityVerifierWithStackTracing(simulations[0], simulations[1], exceptions);

      for (int tickIndex = 0; tickIndex < numTicksToTest; tickIndex++)
      {
         boolean areTheVariableChangesDifferent = helper.areTheVariableChangesDifferent();
         if (areTheVariableChangesDifferent)
         {
            helper.printOutStackTracesOfFirstChangedVariable();
            return;  //DEBUG REWINDABILITY
         }

         if (DEBUG)
            System.out.println("SimulationRewindabilityVerifier: Passed sims match test on loop " + tickIndex + " of " + numTicksToTest);

         helper.clearChangesForSimulations();

         helper.setRecordDifferencesForSimOne(true);
         helper.setRecordDifferencesForSimTwo(false);
         simulations[0].simulateOneRecordStepNow();
         helper.setRecordDifferencesForSimOne(false);

         int numberOfStepsForward = 1;
         for (int i=0; i< numberOfStepsForward; i++)
         {
            simulations[1].simulateOneRecordStepNow();
         }
         for (int i=0; i< numberOfStepsForward; i++)
         {
            simulations[1].stepBackwardNow();
         }

         helper.setRecordDifferencesForSimTwo(true);
         simulations[1].simulateOneRecordStepNow();
      }
   }


   public void simulateForNumberOfTicks(int numberOfTicks) throws UnreasonableAccelerationException
   {
      simulateForNumberOfTicks(simulations[0], numberOfTicks);
      simulateForNumberOfTicks(simulations[1], numberOfTicks);
   }

   private static void simulateForNumberOfTicks(SimulationConstructionSet scs, int numberOfTicks) throws UnreasonableAccelerationException
   {
      for (int i = 0; i < numberOfTicks; i++)
      {
         scs.simulateOneRecordStepNow();
      }
   }

   private boolean verifyMatch(VariablesThatShouldMatchList variablesThatShouldMatchList, ArrayList<VariableDifference> variableDifferencesToPack,
                               double maxDifferenceAllowed)
   {
      double time = simulations[0].getTime();

      boolean checkForPercentDifference = false;

      return variablesThatShouldMatchList.doVariableValuesMatch(variableDifferencesToPack, time, maxDifferenceAllowed, checkForPercentDifference);
   }

   
}
