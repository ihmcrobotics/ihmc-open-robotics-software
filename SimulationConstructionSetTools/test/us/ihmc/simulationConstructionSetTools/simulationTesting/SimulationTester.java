package us.ihmc.simulationConstructionSetTools.simulationTesting;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;

public abstract class SimulationTester
{
   protected static final int NUMBER_OF_SIMULATIONS = 2;
   private static final boolean DESTROY_SIMULATIONS_WHEN_DONE = true;
   
   protected final SimulationComparer simulationComparer;
   protected abstract ArrayList<SimulationConstructionSet> createAndStartSimulations();

   public SimulationTester(SimulationComparer simulationComparer)
   {
      this.simulationComparer = simulationComparer;
   }

   public boolean test(double simulationTime)
   {
      ArrayList<SimulationConstructionSet> simulationConstructionSets = createAndStartSimulations();
      simulate(simulationConstructionSets, simulationTime);
      waitForSimulationsToFinish(simulationConstructionSets);
      
      // Some stupid bug requires us to sleep a little or else things fail every so often...
      try
      {
         Thread.sleep(4000);
      }
      catch (InterruptedException e)
      {
      }
      
      if (!didSimulationsGetToEndTime(simulationTime, simulationConstructionSets))
      {
         System.err.println("Simlulation didn't get to end time!");
         return false;
      }
      
      boolean ret = simulationComparer.compare(simulationConstructionSets.get(0), simulationConstructionSets.get(1));
      
      if (DESTROY_SIMULATIONS_WHEN_DONE)
      {
         for (SimulationConstructionSet scs : simulationConstructionSets)
         {
            scs.closeAndDispose();
         }
      }
      
      return ret;
   }
   
   private boolean didSimulationsGetToEndTime(double simulationTime, ArrayList<SimulationConstructionSet> simulationConstructionSets)
   {
      double endTime0 = simulationConstructionSets.get(0).getRobots()[0].getTime();
      double endTime1 = simulationConstructionSets.get(0).getRobots()[0].getTime();
      
      double errorTime0 = Math.abs(simulationTime - endTime0);
      double errorTime1 = Math.abs(simulationTime - endTime1);
      
//      System.out.println("errorTime0 = " + errorTime0);
//      System.out.println("errorTime1 = " + errorTime1);
      
      if (errorTime0 > 1e-3) return false;
      if (errorTime1 > 1e-3) return false;
      
      return true;
   }
   
   
   public double findTimeOfFirstDifference(double maxSimulationTime)
   {
      ArrayList<SimulationConstructionSet> simulationConstructionSets = createAndStartSimulations();
      boolean differenceFound = false;
      Robot robotToUseForTime = simulationConstructionSets.get(0).getRobots()[0];
      double time = robotToUseForTime.getTime();
      while (!differenceFound && (time < maxSimulationTime))
      {
         for (SimulationConstructionSet scs : simulationConstructionSets)
         {
            try
            {
               scs.simulateOneTimeStep();
            }
            catch (UnreasonableAccelerationException e)
            {
               e.printStackTrace();
            }
         }

         differenceFound = !simulationComparer.compare(simulationConstructionSets.get(0), simulationConstructionSets.get(1));
         time = robotToUseForTime.getTime();
      }

      double ret = differenceFound ? time : Double.NaN;
      
      if (DESTROY_SIMULATIONS_WHEN_DONE)
      {
         for (SimulationConstructionSet scs : simulationConstructionSets)
         {
            scs.closeAndDispose();
         }
      }
      return ret;
   }

   private void simulate(ArrayList<SimulationConstructionSet> simulationConstructionSets, double simulationTime)
   {
      for (SimulationConstructionSet scs : simulationConstructionSets)
      {
         scs.simulate(simulationTime);
      }
   }

   private void waitForSimulationsToFinish(ArrayList<SimulationConstructionSet> simulationConstructionSets)
   {
      boolean allDone = false;
      while (!allDone)
      {
         allDone = true;

         for (SimulationConstructionSet scs : simulationConstructionSets)
         {
            if (scs.isSimulating())
            {
               allDone = false;
            }
         }
      }
   }
   

}
