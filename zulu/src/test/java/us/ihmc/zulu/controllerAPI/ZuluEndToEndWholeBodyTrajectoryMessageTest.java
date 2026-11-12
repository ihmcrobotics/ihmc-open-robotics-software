package us.ihmc.zulu.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndWholeBodyTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class ZuluEndToEndWholeBodyTrajectoryMessageTest extends EndToEndWholeBodyTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   // TRIAL: forcing the Mujoco physics engine (instead of the default contact-point engine) for
   // controller-api-2 tests only. See conversation with Nick 2026-08-17.
   private boolean useMujocoPhysicsEngine = false;

   @Override
   protected void configureSimulationFactory(SCS2AvatarTestingSimulationFactory testSimulationFactory)
   {
      if (useMujocoPhysicsEngine)
      {
         testSimulationFactory.setUseMujocoPhysicsEngine(true);
         testSimulationFactory.setSimulationDT(0.001);
      }
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testIssue47BadChestTrajectoryMessage() throws Exception
   {
      super.testIssue47BadChestTrajectoryMessage();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testIssue47BadPelvisTrajectoryMessage() throws Exception
   {
      super.testIssue47BadPelvisTrajectoryMessage();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypoint() throws Exception
   {
      useMujocoPhysicsEngine = true;
      super.testSingleWaypoint();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testSingleWaypointUsingMessageOfMessages() throws Exception
   {
      super.testSingleWaypointUsingMessageOfMessages();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testSingleWaypointUsingMessageOfMessagesWithDelays() throws Exception
   {
      super.testSingleWaypointUsingMessageOfMessagesWithDelays();
   }
}
