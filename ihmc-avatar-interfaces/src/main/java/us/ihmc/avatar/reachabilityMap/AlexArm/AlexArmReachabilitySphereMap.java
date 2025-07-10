package us.ihmc.avatar.reachabilityMap.AlexArm;

import us.ihmc.avatar.reachabilityMap.ReachabilityMapRobotInformation;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper;
import us.ihmc.avatar.reachabilityMap.AlexArm.AlexArmParameters.AlexArmLinkParameters;

public class AlexArmReachabilitySphereMap
{
    public AlexArmReachabilitySphereMap()
    {
        AlexArmDefinition robotDefinition = new AlexArmDefinition();
        ReachabilityMapRobotInformation robotInformation = new ReachabilityMapRobotInformation(robotDefinition,
                robotDefinition.getRootBodyDefinition().getName(),
                AlexArmLinkParameters.getEndEffector().getLinkName());
        ReachabilitySphereMapSimulationHelper simHelper = new ReachabilitySphereMapSimulationHelper(robotInformation);
        simHelper.setGridParameters(25, 0.025, 50, 3);

        simHelper.start();
    }

    public static void main(String[] args)
    {
        new AlexArmReachabilitySphereMap();
    }
}
