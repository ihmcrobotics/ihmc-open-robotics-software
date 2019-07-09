package us.ihmc.atlas.kinematicsSimulation;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.AtlasBehaviorTestYoVariables;
import us.ihmc.avatar.AvatarTestYoVariables;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.AvatarKinematicsSimulationTest;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AtlasKinematicsSimulationTest extends AvatarKinematicsSimulationTest
{
   // under construction


   @Override
   public AvatarTestYoVariables createTestYoVariables(SimulationConstructionSet scs)
   {
      return new AtlasBehaviorTestYoVariables(scs);
   }

   @Override
   public DRCRobotModel createRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public SimulationConstructionSet createSCS()
   {
      return null; // TODO
   }
}
