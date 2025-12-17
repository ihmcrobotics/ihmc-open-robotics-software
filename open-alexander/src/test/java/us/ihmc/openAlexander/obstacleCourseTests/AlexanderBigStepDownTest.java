package us.ihmc.openAlexander.obstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.openAlexander.parameters.controller.OpenAlexanderWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.obstacleCourseTests.AvatarBigStepDownTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

/**
 * Tests in this class have been replaced with Nadia tests so they are all disabled
 */
@Tag("humanoid-obstacle")
public class AlexanderBigStepDownTest extends AvatarBigStepDownTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new OpenAlexanderWalkingControllerParameters(getRobotVersion(), getTarget(), getJointMap(), getPhysicalProperties())
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
