package us.ihmc.zulu.networkProcessor.walkingPreview;

import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.walkingPreview.AvatarWalkingControllerPreviewToolboxControllerTest;

public class ZuluWalkingControllerPreviewToolboxControllerTest extends AvatarWalkingControllerPreviewToolboxControllerTest
{
   private final ZuluRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

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
