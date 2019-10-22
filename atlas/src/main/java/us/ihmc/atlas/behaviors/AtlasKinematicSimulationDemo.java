package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;

public class AtlasKinematicSimulationDemo
{
   public AtlasKinematicSimulationDemo()
   {
      HumanoidKinematicsSimulation.createForManualTest(createRobotModel(), true);
   }

   private AtlasRobotModel createRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   public static void main(String[] args)
   {
      new AtlasKinematicSimulationDemo();
   }
}
