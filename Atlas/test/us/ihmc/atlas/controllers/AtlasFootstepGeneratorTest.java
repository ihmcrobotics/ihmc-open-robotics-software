package us.ihmc.atlas.controllers;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotBasedFootstepGeneratorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.CustomJob;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.CustomJobType;

@CustomJob(job = CustomJobType.Utilities)
public class AtlasFootstepGeneratorTest extends DRCRobotBasedFootstepGeneratorTest
{
   private static final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.DRC_NO_HANDS;
   private final static DRCRobotModel robotModel = new AtlasRobotModel(ATLAS_ROBOT_VERSION, AtlasRobotModel.AtlasTarget.SIM, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

}
