package us.ihmc.atlas;


import us.ihmc.darpaRoboticsChallenge.DRCHumanoidBehaviorICPFaultDetectionTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;


@BambooPlan(planType = {BambooPlanType.InDevelopment})
public class HumanoidBehaviorsICPFaultDetectionTest extends DRCHumanoidBehaviorICPFaultDetectionTest
{  
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }

   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
