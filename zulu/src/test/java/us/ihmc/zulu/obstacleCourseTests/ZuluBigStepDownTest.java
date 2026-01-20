package us.ihmc.zulu.obstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.parameters.controller.ZuluWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.obstacleCourseTests.AvatarBigStepDownTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

/**
 * Tests in this class have been replaced with Nadia tests so they are all disabled
 */
@Tag("humanoid-obstacle")
public class ZuluBigStepDownTest extends AvatarBigStepDownTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ZuluWalkingControllerParameters(getRobotVersion(), getTarget(), getJointMap(), getPhysicalProperties())
            {
               @Override
               public double nominalHeightAboveAnkle()
               {
                  return 0.849;
               }
            };
         }
      };

      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Tag("humanoid-obstacle")
   @Disabled
   @Test
   public void testSplitFractionInBigStepDown()
   {
      super.testSplitFractionInBigStepDown();
   }

   @Tag("humanoid-obstacle")
   @Disabled
   @Test
   public void testWalkingOffOfLargePlatform()
   {
      super.testWalkingOffOfLargePlatform();
   }
}
