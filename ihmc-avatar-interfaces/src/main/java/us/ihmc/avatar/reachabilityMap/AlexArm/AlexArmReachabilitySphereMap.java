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

        /* Voxel grid dimensions */
        int gridSizeInNumberOfVoxels = 40;
        double voxelSize = 0.025;

        /* Ray Reachability */
        int numberOfRays = 50;
        simHelper.setEvaluateRReachability(true);

        /* Pose Reachability */
        simHelper.setEvaluateR2Reachability(true);
        int numberOfRotationsAroundRay = 3;

        simHelper.setGridParameters(gridSizeInNumberOfVoxels, voxelSize, numberOfRays, numberOfRotationsAroundRay);

        simHelper.start();
    }

    public static void main(String[] args)
    {
        new AlexArmReachabilitySphereMap();
    }
}
