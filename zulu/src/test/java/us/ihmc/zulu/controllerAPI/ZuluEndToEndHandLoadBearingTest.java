package us.ihmc.zulu.controllerAPI;

import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import org.junit.jupiter.api.Disabled;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndHandLoadBearingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import static org.junit.jupiter.api.Assertions.*;

public class ZuluEndToEndHandLoadBearingTest extends EndToEndHandLoadBearingTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   protected double getPelvisHeightOffset()
   {
      return -0.1;
   }

   @Override
   protected double getHandForwardPositionInChest()
   {
      return 0.4;
   }

   @Override
   protected void applyPitch(ReferenceFrame pelvisZUpFrame)
   {
      Quaternion pelvisOrientation = new Quaternion();
      pelvisOrientation.appendPitchRotation(Math.PI / 4.0);
      PelvisOrientationTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(1.0, pelvisOrientation, ReferenceFrame.getWorldFrame());
      simulationTestHelper.publishToController(pelvisTrajectoryMessage);
      boolean success = simulationTestHelper.simulateNow(1.5);
      assertTrue(success);
   }

//   @Tag("controller-api")
//   @Override
   @Disabled
   public void testUsingHand() throws SimulationExceededMaximumTimeException
   {
      super.testUsingHand();
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
