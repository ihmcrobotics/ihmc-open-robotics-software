package us.ihmc.openAlexander.networkProcessor.walkingPreview;

import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.walkingPreview.AvatarWalkingControllerPreviewToolboxControllerTest;

public class AlexanderWalkingControllerPreviewToolboxControllerTest extends AvatarWalkingControllerPreviewToolboxControllerTest
{
   private final OpenAlexanderRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   @Test
   public void testWalkingPreviewAlone()
   {
      super.testWalkingPreviewAlone();
   }

   @Override
   @Test
   public void testStepsInPlacePreviewAtControllerDT()
   {
      super.testStepsInPlacePreviewAtControllerDT();
   }

   @Override
   @Test
   public void testResetFeature()
   {
      super.testResetFeature();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }
}
