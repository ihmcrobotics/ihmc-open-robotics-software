package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.DRCBumpyAndShallowRampsWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;

@BambooPlan(planType = {BambooPlanType.Fast, BambooPlanType.VideoA})
public class AtlasBumpyAndShallowRampsWalkingTest extends DRCBumpyAndShallowRampsWalkingTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

}
