package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.dynamicsSimulation.HumanoidDynamicsSimulation;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

public class AtlasDynamicsSimulation
{
   public static void main(String[] args)
   {
      int recordTicksPerControllerTick = 1;
      int dataBufferSize = 1024;
      HumanoidDynamicsSimulation.createForManualTest(new AtlasRobotModel(AtlasBehaviorModule.ATLAS_VERSION, RobotTarget.SCS, false),
                                                     new FlatGroundEnvironment(),
                                                     recordTicksPerControllerTick,
                                                     dataBufferSize).simulate();
   }
}
