package us.ihmc.simulationconstructionset.util.simulationRunner;

import java.util.ArrayList;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

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
      return checkRewindabilityWithSimpleMethod(1, numTicksToTest, maxDifferenceAllowed);
   }
   
   /**
    * This is a simple rewindability checker. For the first simulation it ticks it ahead one tick at a time.
    * For the second simulation it ticks it ahead a tick, backs up a tick, and ticks ahead another tick.
    * Then it compares the two simulations. It repeats this for the indicated number of times.
    * Any differences are put in the returned ArrayList.
    */
   public ArrayList<VariableDifference> checkRewindabilityWithSimpleMethod(int numTicksToStartComparingAt, int numTicksToTest, double maxDifferenceAllowed)
           throws UnreasonableAccelerationException
   {
      YoVariableRegistry registry0 = simulations[0].getRootRegistry();
      YoVariableRegistry registry1 = simulations[1].getRootRegistry();

      VariablesThatShouldMatchList variablesThatShouldMatchList = new VariablesThatShouldMatchList(registry0, registry1, exceptions);
      ArrayList<VariableDifference> variableDifferencesToReturn = new ArrayList<VariableDifference>();

      for (int i=0; i<numTicksToStartComparingAt; i++)
      {
         simulations[0].simulateOneRecordStepNow();
      }   
      
      for (int i=0; i<numTicksToStartComparingAt; i++)
      {
         simulations[1].simulateOneRecordStepNow();
      }   
      
      for (int i = 0; i < numTicksToTest; i++)
      {
         boolean passesTest = verifyMatch(variablesThatShouldMatchList, variableDifferencesToReturn, maxDifferenceAllowed);
         if (!passesTest)
         {
            return variableDifferencesToReturn;
         }
         
         if (DEBUG) System.out.println("SimulationRewindabilityVerifier: Passed sims match test on loop " + i + " of " + numTicksToTest);

         simulations[0].simulateOneRecordStepNow();
         simulations[1].simulateOneRecordStepNow();

         simulations[1].stepBackwardNow();
         simulations[1].simulateOneRecordStepNow();
      }

      return variableDifferencesToReturn;
   }



   /**
    * This is a more rigorous rewindability checker. For the first simulation it ticks it ahead one tick at a time.
    * For the second simulation it ticks it ahead numTicksToSimulateAhead ticks, rewinds it to where it had started,
    * and ticks ahead another tick.
    * Then it compares the two simulations. It repeats this for the indicated number of numTicksToTest.
    * Any differences are put in the returned ArrayList.
    */
   public ArrayList<VariableDifference> checkRewindabilityWithRigorousMethod(int numTicksToTest, int numTicksToSimulateAhead, double maxDifferenceAllowed)
           throws UnreasonableAccelerationException
   {
      YoVariableRegistry registry0 = simulations[0].getRootRegistry();
      YoVariableRegistry registry1 = simulations[1].getRootRegistry();

      VariablesThatShouldMatchList variablesThatShouldMatchList = new VariablesThatShouldMatchList(registry0, registry1, exceptions);

      ArrayList<VariableDifference> variableDifferencesToReturn = new ArrayList<VariableDifference>();

      simulations[0].simulateOneRecordStepNow();
      simulations[1].simulateOneRecordStepNow();
      
      for (int i = 0; i < numTicksToTest; i++)
      {
         boolean passesTest = verifyMatch(variablesThatShouldMatchList, variableDifferencesToReturn, maxDifferenceAllowed);
         if (!passesTest)
         {
            return variableDifferencesToReturn;
         }

         if (DEBUG) System.out.println("SimulationRewindabilityVerifier: Passed sims match test on loop " + i + " of " + numTicksToTest);

         
         if (DEBUG) System.out.println("SimulationRewindabilityVerifier: Simulating one Record Step on Sim0.");

         simulations[0].simulateOneRecordStepNow();
         simulations[1].setInPoint();

         if (DEBUG) System.out.println("SimulationRewindabilityVerifier: Simulating " + numTicksToSimulateAhead + " Record Steps on Sim1.");

         for (int j = 0; j < numTicksToSimulateAhead; j++)
         {
            simulations[1].simulateOneRecordStepNow();
         }

         if (DEBUG) System.out.println("SimulationRewindabilityVerifier: Going to InPoint and Simulating one Record Step on Sim1.");
        
         simulations[1].gotoInPointNow();
         simulations[1].simulateOneRecordStepNow();
      }

      return variableDifferencesToReturn;
   }



   private boolean verifyMatch(VariablesThatShouldMatchList variablesThatShouldMatchList, ArrayList<VariableDifference> variableDifferencesToPack,
                               double maxDifferenceAllowed)
   {
      double time = simulations[0].getTime();

      boolean checkForPercentDifference = false;

      return variablesThatShouldMatchList.doVariableValuesMatch(variableDifferencesToPack, time, maxDifferenceAllowed, checkForPercentDifference);
   }


// private void waitForSimulationToFinish(SimulationConstructionSet scs)
// {
//    while (scs.isRunning())
//    {
//       try
//       {
//          Thread.sleep(10);
//       }
//       catch (InterruptedException e)
//       {
//       }
//    }
// }

}
