package us.ihmc.openAlexander.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndSpineJointTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class AlexanderEndToEndSpineJointTrajectoryMessageTest extends EndToEndSpineJointTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypoint()
   {
      super.testSingleWaypoint();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testSwitchingBetweenControlModes()
   {
      super.testSwitchingBetweenControlModes();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testDesiredsAreContinuous()
   {
      super.testDesiredsAreContinuous();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testMultipleWaypoints()
   {
      super.testMultipleWaypoints();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testLongMessage()
   {
      super.testLongMessage();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testMessageQueuing()
   {
      super.testMessageQueuing();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testMessageWithDifferentTrajectoryLengthsPerJoint()
   {
      super.testMessageWithDifferentTrajectoryLengthsPerJoint();
   }

   @Tag("controller-api-2")
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
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ALEXANDER);
   }

}
