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
        int gridSizeInNumberOfVoxels = 25;
        double voxelSize = 0.025;

        /* Set to true enables ray reachability, i.e. testing N rays per position, with each test having a free orientation dof around the ray */
        int numberOfRays = 50;
        simHelper.setEvaluateRReachability(true);

        /* Set to true enables pose reachability, i.e. testing M orientations per ray, with each test specifying a full 6 dof pose */
        simHelper.setEvaluateR2Reachability(true);
        int numberOfRotationsAroundRay = 6;

        simHelper.setGridParameters(gridSizeInNumberOfVoxels, voxelSize, numberOfRays, numberOfRotationsAroundRay);
        simHelper.start();
    }

    public static void main(String[] args)
    {
        new AlexArmReachabilitySphereMap();
    }
}
