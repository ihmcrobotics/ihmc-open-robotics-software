package us.ihmc.zulu.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisOrientationTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class ZuluEndToEndPelvisOrientationTest extends EndToEndPelvisOrientationTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Tag("controller-api-2")
   @Override
   @Test
   public void testGoHome()
   {
      super.testGoHome();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleTrajectoryPoint()
   {
      super.testSingleTrajectoryPoint();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testQueue()
   {
      super.testQueue();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalking()
   {
      super.testWalking();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingAfterTrajectory()
   {
      super.testWalkingAfterTrajectory();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testMultipleTrajectoryPoints()
   {
      super.testMultipleTrajectoryPoints();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingWithUserControl()
   {
      super.testWalkingWithUserControl();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testCustomControlFrame()
   {
      super.testCustomControlFrame();
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
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   // TRIAL: forcing the Mujoco physics engine (instead of the default contact-point engine) for
   // controller-api-2 tests only. See conversation with Nick 2026-08-17.
   @Override
   protected void configureSimulationFactory(SCS2AvatarTestingSimulationFactory testSimulationFactory, TestInfo testInfo)
   {
      if (testInfo.getTags().contains("controller-api-2"))
      {
         testSimulationFactory.setUseMujocoPhysicsEngine(true);
         testSimulationFactory.setSimulationDT(0.001);
      }
   }
}
