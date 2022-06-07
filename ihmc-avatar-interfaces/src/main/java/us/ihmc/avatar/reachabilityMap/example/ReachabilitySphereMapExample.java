package us.ihmc.avatar.reachabilityMap.example;

import us.ihmc.avatar.reachabilityMap.ReachabilityMapRobotInformation;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper;
import us.ihmc.avatar.reachabilityMap.example.RobotParameters.RobotArmLinkParameters;

public class ReachabilitySphereMapExample
{
   public ReachabilitySphereMapExample()
   {
      RobotArmDefinition robotDefinition = new RobotArmDefinition();
      ReachabilityMapRobotInformation robotInformation = new ReachabilityMapRobotInformation(robotDefinition,
                                                                                             robotDefinition.getRootBodyDefinition().getName(),
                                                                                             RobotArmLinkParameters.getEndEffector().getLinkName());
      ReachabilitySphereMapSimulationHelper simHelper = new ReachabilitySphereMapSimulationHelper(robotInformation);
      simHelper.setGridParameters(25, 0.025, 50, 1);

      simHelper.start();
   }

   public static void main(String[] args)
   {
      new ReachabilitySphereMapExample();
   }
}
