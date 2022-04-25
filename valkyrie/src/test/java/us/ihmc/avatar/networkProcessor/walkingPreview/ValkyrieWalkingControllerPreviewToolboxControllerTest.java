package us.ihmc.avatar.networkProcessor.walkingPreview;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieWalkingControllerPreviewToolboxControllerTest extends AvatarWalkingControllerPreviewToolboxControllerTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

   @Tag("humanoid-toolbox")
   @Override
   @Test
   public void testWalkingPreviewAlone()
   {
      super.testWalkingPreviewAlone();
   }

   @Tag("humanoid-toolbox")
   @Override
   @Test
   public void testStepsInPlacePreviewAtControllerDT()
   {
      super.testStepsInPlacePreviewAtControllerDT();
   }

   @Tag("humanoid-toolbox")
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
