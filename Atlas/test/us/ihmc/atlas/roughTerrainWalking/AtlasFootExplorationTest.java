package us.ihmc.atlas.roughTerrainWalking;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.roughTerrainWalking.DRCFootExplorationTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;

@BambooPlan(planType = {BambooPlanType.Slow, BambooPlanType.VideoA})
public class AtlasFootExplorationTest extends DRCFootExplorationTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);

      AtlasContactPointParameters contactPointParameters = (AtlasContactPointParameters) robotModel.getContactPointParameters();
      contactPointParameters.addMoreFootContactPointsSimOnly();
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
