package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.jumpingSimulation.JumpingSimulationFactory;

public class AtlasJumpingSimulation
{
   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      JumpingSimulationFactory simulationFactory = new JumpingSimulationFactory(robotModel);
      simulationFactory.createSimulation();
   }
}
