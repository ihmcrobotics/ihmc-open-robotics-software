package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.controllerAPI.EndToEndAdjustFootstepMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndAdjustFootstepMessageTest extends EndToEndAdjustFootstepMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @Test
   public void testAdjustFootstepOnce() throws Exception
   {
      super.testAdjustFootstepOnce();
   }
}
