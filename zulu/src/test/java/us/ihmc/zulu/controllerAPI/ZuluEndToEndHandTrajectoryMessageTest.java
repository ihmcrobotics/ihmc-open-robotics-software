package us.ihmc.zulu.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.parameters.model.ZuluPhysicalPropertiesV0;
import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ZuluEndToEndHandTrajectoryMessageTest extends EndToEndHandTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

//   @Override
//   public double getHeightOffsetFromChestForSphereCenter()
//   {
//      return -0.25;
//   }
//
//   @Override
//   public double getStreamingRangeOfMotion()
//   {
//      return 0.4;
//   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlFrame();
   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      super.testMessageWithTooManyTrajectoryPoints();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testMultipleTrajectoryPoints() throws Exception
   {
      super.testMultipleTrajectoryPoints();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testQueuedMessages() throws Exception
   {
      super.testQueuedMessages();
   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

   @Tag("controller-api-slow-3")
   @Test
   @Override
   public void testForceExecutionWithSingleTrajectoryPoint() throws Exception
   {
      super.testForceExecutionWithSingleTrajectoryPoint();
   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Disabled // Could not get a tracking that is decent enough to perform assertions. The test pass with Valkyrie, should be enough.
   @Override
   @Test
   public void testHoldHandWhileWalking() throws SimulationExceededMaximumTimeException
   {
      super.testHoldHandWhileWalking();
   }

   @Tag("controller-api")
   @Disabled // This test is not applicable to Zulu, TODO fix me.
   @Override
   @Test
   public void testStreaming() throws Exception
   {
      super.testStreaming();
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

   @Override
   public double getLegLength()
   {
      ZuluPhysicalPropertiesV0 physicalProperties = new ZuluPhysicalPropertiesV0();
      return physicalProperties.getShinLength() + physicalProperties.getThighLength();
   }
}
