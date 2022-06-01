package us.ihmc.atlas.kinematicsSimulation;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.AtlasBehaviorTestYoVariables;
import us.ihmc.avatar.AvatarTestYoVariables;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationTest;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AtlasKinematicsSimulationTest extends HumanoidKinematicsSimulationTest
{
   // under construction


   @Override
   public AvatarTestYoVariables createTestYoVariables(SimulationConstructionSet2 scs)
   {
      return new AtlasBehaviorTestYoVariables(scs);
   }

   @Override
   public DRCRobotModel createRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public SimulationConstructionSet2 createSCS()
   {
      return null; // TODO
   }
}
