package us.ihmc.atlas;


import us.ihmc.avatar.DRCHumanoidBehaviorICPFaultDetectionTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;


@ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
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
