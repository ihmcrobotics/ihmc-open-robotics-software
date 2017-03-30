package us.ihmc.simulationConstructionSetTools.simulationTesting;

import java.util.ArrayList;

import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.StateFileComparer;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.tools.thread.ThreadTools;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SimulationRunsSameWayTwiceVerifier
{
   private final SimulationConstructionSet scsOne, scsTwo;
   private final BlockingSimulationRunner blockingSimulationRunnerOne, blockingSimulationRunnerTwo;
   private final double walkingTimeDuration, standingTimeDuration;

   public SimulationRunsSameWayTwiceVerifier(SimulationConstructionSet scsOne, SimulationConstructionSet scsTwo, double standingTimeDuration, double walkingTimeDuration)
   {
      this.scsOne = scsOne;
      this.scsTwo = scsTwo;
      this.standingTimeDuration = standingTimeDuration;
      this.walkingTimeDuration = walkingTimeDuration;

      this.blockingSimulationRunnerOne = new BlockingSimulationRunner(scsOne, 3000.0);
      this.blockingSimulationRunnerTwo = new BlockingSimulationRunner(scsTwo, 3000.0);

      ThreadTools.sleep(1000);
   }

   public boolean verifySimRunsSameWayTwice(double maxPercentDifference, ArrayList<String> stringsToIgnore) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, ControllerFailureException

   {
      boolean firstSimDone, secondSimDone;
      firstSimDone = secondSimDone = false;


      initiateMotion(scsOne, standingTimeDuration, blockingSimulationRunnerOne);
      initiateMotion(scsTwo, standingTimeDuration, blockingSimulationRunnerTwo);

      while(!firstSimDone && !secondSimDone)
      {
         blockingSimulationRunnerOne.simulateAndBlock(1.0);
         blockingSimulationRunnerTwo.simulateAndBlock(1.0);

         if (!compareSCSInstances(maxPercentDifference, stringsToIgnore))
         {
            System.err.println("Mismatch in sim states!");
            return false;
         }

         firstSimDone = (scsOne.getTime() - standingTimeDuration) >= walkingTimeDuration;
         secondSimDone = (scsTwo.getTime() - standingTimeDuration) >= walkingTimeDuration;

         if(firstSimDone != secondSimDone)
         {
            System.err.println("Sims did not finish at the same time!");
            return false;
         }
      }

      return true;
   }

   private boolean compareSCSInstances(double maxPercentDifference, ArrayList<String> stringsToIgnore)
   {
      YoVariableList listOne = new YoVariableList("SimOneList");
      YoVariableList listTwo = new YoVariableList("SimTwoList");

      listOne.addVariables(scsOne.getAllVariables());
      listTwo.addVariables(scsTwo.getAllVariables());

      ArrayList<VariableDifference> variableDifferences = StateFileComparer.compareVarLists(listOne, listTwo, maxPercentDifference, true, stringsToIgnore);
      if(variableDifferences.isEmpty())
      {
         return true;
      }

      for(VariableDifference difference : variableDifferences)
      {
         System.err.println("Variable One: " + difference.getVariableOne());
         System.err.println("Variable Two: " + difference.getVariableTwo());
         System.err.println(difference.toString());
      }

      return false;
   }

   private void initiateMotion(SimulationConstructionSet scs, double standingTimeDuration, BlockingSimulationRunner runner) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, ControllerFailureException

   {
      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("walk");
      if(walk != null)
      {
         walk.set(false);
      }
      runner.simulateAndBlock(standingTimeDuration);
      if(walk != null)
      {
         walk.set(true);
      }
   }
}
