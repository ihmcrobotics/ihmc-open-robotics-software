package us.ihmc.atlas;


import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.CustomJob;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.CustomJobType;
@CustomJob(job = CustomJobType.PushRecovery)
public class AtlasPushRecoveryWalkingTest extends DRCPushRecoveryWalkingTest
{  
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
