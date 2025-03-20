package us.ihmc.alexander.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.AlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndGoHomeMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AlexanderEndToEndGoHomeMessageTest extends EndToEndGoHomeMessageTest
{
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(AlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);

   @Tag("controller-api")
   @Override
   @Test
   public void testGoHomeArms() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomeArms();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testGoHomeChest() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomeChest();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testGoHomePelvis() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomePelvis();
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
