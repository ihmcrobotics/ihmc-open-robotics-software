package us.ihmc.zulu.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisHeightTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ZuluEndToEndPelvisHeightTrajectoryMessageTest extends EndToEndPelvisHeightTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

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

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypoint() throws Exception
   {
      useMujocoPhysicsEngine = true;
      super.testSingleWaypoint();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypointInUserMode() throws Exception
   {
      useMujocoPhysicsEngine = true;
      super.testSingleWaypointInUserMode();
   }

   /*
    * FIXME This test wasn't running for a very long time and failed when re-enabled. Need to fix it
    * and re-enbable it.
    */
   @Disabled
   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testSingleWaypointThenManualChange() throws Exception
   {
      super.testSingleWaypointThenManualChange();
   }

   /*
    * FIXME This test wasn't running for a very long time and failed when re-enabled. Need to fix it
    * and re-enbable it.
    */
   @Disabled
   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testSingleWaypointWithControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypointWithControlFrame();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testStreaming() throws Exception
   {
      useMujocoPhysicsEngine = true;
      super.testStreaming();
   }
}
