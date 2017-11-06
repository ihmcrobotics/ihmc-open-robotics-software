package us.ihmc.valkyrie.controllerAPI;

import org.junit.Test;

import us.ihmc.avatar.controllerAPI.EndToEndEndFootBearingMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndFootLoadBearingMessageTest extends EndToEndEndFootBearingMessageTest
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
   @ContinuousIntegrationTest(estimatedDuration = 41.3)
   @Test(timeout = 80000)
   public void testSwitchFootToLoadBearing() throws Exception
   {
      super.testSwitchFootToLoadBearing();
   }
}
