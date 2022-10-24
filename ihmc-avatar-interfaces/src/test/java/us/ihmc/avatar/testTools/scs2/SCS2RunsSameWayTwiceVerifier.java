package us.ihmc.avatar.testTools.scs2;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.StateFileComparer;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoBoolean;

public class SCS2RunsSameWayTwiceVerifier
{
   private final SCS2AvatarTestingSimulation simulationTestHelperOne, simulationTestHelperTwo;
   private final double walkingTimeDuration, standingTimeDuration;

   public SCS2RunsSameWayTwiceVerifier(SCS2AvatarTestingSimulation simulationTestHelperOne,
                                       SCS2AvatarTestingSimulation simulationTestHelperTwo,
                                       double standingTimeDuration,
                                       double walkingTimeDuration)
   {
      this.simulationTestHelperOne = simulationTestHelperOne;
      this.simulationTestHelperTwo = simulationTestHelperTwo;
      this.standingTimeDuration = standingTimeDuration;
      this.walkingTimeDuration = walkingTimeDuration;

      ThreadTools.sleep(1000);
   }

   public boolean verifySimRunsSameWayTwice(double maxPercentDifference, List<String> stringsToIgnore)
   {
      boolean firstSimDone, secondSimDone;
      firstSimDone = secondSimDone = false;

      initiateMotion(simulationTestHelperOne, standingTimeDuration);
      initiateMotion(simulationTestHelperTwo, standingTimeDuration);

      while (!firstSimDone && !secondSimDone)
      {
         simulationTestHelperOne.simulateNow(1.0);
         simulationTestHelperTwo.simulateNow(1.0);

         if (!compareSCSInstances(maxPercentDifference, stringsToIgnore))
         {
            System.err.println("Mismatch in sim states!");
            return false;
         }

         firstSimDone = (simulationTestHelperOne.getSimulationTime() - standingTimeDuration) >= walkingTimeDuration;
         secondSimDone = (simulationTestHelperTwo.getSimulationTime() - standingTimeDuration) >= walkingTimeDuration;

         if (firstSimDone != secondSimDone)
         {
            System.err.println("Sims did not finish at the same time!");
            return false;
         }
      }

      return true;
   }

   private boolean compareSCSInstances(double maxPercentDifference, List<String> stringsToIgnore)
   {
      YoVariableList listOne = new YoVariableList("SimOneList");
      YoVariableList listTwo = new YoVariableList("SimTwoList");

      listOne.addAll(simulationTestHelperOne.getVariables());
      listTwo.addAll(simulationTestHelperTwo.getVariables());

      ArrayList<VariableDifference> variableDifferences = StateFileComparer.compareVarLists(listOne, listTwo, maxPercentDifference, true, stringsToIgnore);
      if (variableDifferences.isEmpty())
      {
         return true;
      }

      for (VariableDifference difference : variableDifferences)
      {
         System.err.println("Variable One: " + difference.getVariableOne());
         System.err.println("Variable Two: " + difference.getVariableTwo());
         System.err.println(difference.toString());
      }

      return false;
   }

   private void initiateMotion(SCS2AvatarTestingSimulation simulationTestHelper, double standingTimeDuration)
   {
      YoBoolean walk = (YoBoolean) simulationTestHelper.findVariable("walkCSG");
      if (walk != null)
      {
         walk.set(false);
      }
      simulationTestHelperTwo.simulateNow(standingTimeDuration);
      if (walk != null)
      {
         walk.set(true);
      }
   }
}
