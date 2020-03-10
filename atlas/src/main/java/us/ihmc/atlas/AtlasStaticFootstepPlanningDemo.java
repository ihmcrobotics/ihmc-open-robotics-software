package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.StaticFootstepPlanningEnvironment;

public class AtlasStaticFootstepPlanningDemo
{
   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      CommonAvatarEnvironmentInterface environment = new StaticFootstepPlanningEnvironment();

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);

      boolean automaticallySimulate = false;

      simulationStarter.startSimulation(null, automaticallySimulate);
   }
}
