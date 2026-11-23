package us.ihmc.zulu.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndNeckTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class ZuluEndToEndNeckTrajectoryMessageTest extends EndToEndNeckTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   // TRIAL: forcing the Mujoco physics engine (instead of the default contact-point engine)
   // to see whether this controller-api-2 test still passes. See conversation with Nick 2026-08-17.
   @Override
   protected void configureSimulationFactory(SCS2AvatarTestingSimulationFactory testSimulationFactory)
   {
      testSimulationFactory.setUseMujocoPhysicsEngine(true);
      testSimulationFactory.setSimulationDT(0.001);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypoint() throws Exception
   {
      super.testSingleWaypoint();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }
}
