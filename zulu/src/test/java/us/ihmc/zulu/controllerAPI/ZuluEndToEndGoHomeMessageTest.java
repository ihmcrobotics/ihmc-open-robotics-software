package us.ihmc.zulu.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndGoHomeMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ZuluEndToEndGoHomeMessageTest extends EndToEndGoHomeMessageTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Tag("controller-api")
   @Override
   @Test
   public void testGoHomeArms() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomeArms();
   }

   // This test is disabled because it assumes that the chest is controlled in taskspace by default. Zulu is controlled in jointspace.
   @Disabled
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
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }
}
