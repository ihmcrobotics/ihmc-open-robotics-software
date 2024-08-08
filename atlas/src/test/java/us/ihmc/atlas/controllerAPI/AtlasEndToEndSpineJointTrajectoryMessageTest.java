package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndSpineJointTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AtlasEndToEndSpineJointTrajectoryMessageTest extends EndToEndSpineJointTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

}
