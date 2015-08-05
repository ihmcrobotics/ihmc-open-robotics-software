package us.ihmc.communication.producers;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.humanoidRobotics.model.FullRobotModel;

@BambooPlan(planType = BambooPlanType.Flaky)
public class AtlasRobotConfigurationDataBufferTest extends RobotConfigurationDataBufferTest
{

   @Override
   public FullRobotModel getFullRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, AtlasTarget.SIM, false).createFullRobotModel();
   }

}