package us.ihmc.zulu.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndSpineJointTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class ZuluEndToEndSpineJointTrajectoryMessageTest extends EndToEndSpineJointTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypoint()
   {
      useMujocoPhysicsEngine = true;
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
      useMujocoPhysicsEngine = true;
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
      useMujocoPhysicsEngine = true;
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
      useMujocoPhysicsEngine = true;
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

}
