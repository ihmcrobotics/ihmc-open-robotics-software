package us.ihmc.atlas;


import us.ihmc.darpaRoboticsChallenge.DRCHumanoidBehaviorICPFaultDetectionTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanTarget;


@DeployableTestClass(targets = {TestPlanTarget.InDevelopment})
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
