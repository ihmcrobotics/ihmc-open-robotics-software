package us.ihmc.atlas;


import org.junit.jupiter.api.Disabled;

import us.ihmc.avatar.DRCHumanoidBehaviorICPFaultDetectionTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;


@Disabled
public class HumanoidBehaviorsICPFaultDetectionTest extends DRCHumanoidBehaviorICPFaultDetectionTest
{  
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
