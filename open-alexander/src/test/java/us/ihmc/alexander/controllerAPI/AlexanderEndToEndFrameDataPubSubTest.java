package us.ihmc.alexander.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.AlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndFrameDataPublisherTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AlexanderEndToEndFrameDataPubSubTest extends EndToEndFrameDataPublisherTest
{
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(AlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);

   @Tag("controller-api")
   @Test
   @Override
   public void testQueuing() throws SimulationExceededMaximumTimeException
   {
      super.testQueuing();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ALEXANDER);
   }

}
