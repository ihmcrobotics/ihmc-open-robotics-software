package us.ihmc.atlas.networkProcessor.walkingPreview;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.walkingPreview.AvatarWalkingControllerPreviewToolboxControllerTest;

public class AtlasWalkingControllerPreviewToolboxControllerTest extends AvatarWalkingControllerPreviewToolboxControllerTest
{
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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
