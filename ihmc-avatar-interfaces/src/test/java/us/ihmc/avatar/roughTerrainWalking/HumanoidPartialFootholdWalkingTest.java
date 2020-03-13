package us.ihmc.avatar.roughTerrainWalking;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

import java.awt.*;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.robotics.Assert.assertTrue;

public abstract class HumanoidPartialFootholdWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Test
   public void testSteppingOntoBlock() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      double blockWidth = 0.3;
      double blockDistanceFromOrigin = 0.2;
      double topHeight = 0.1;
      SimpleBlockEnvironment environment = new SimpleBlockEnvironment(blockDistanceFromOrigin + blockWidth * 0.5, blockWidth, 0.4, topHeight);

      setupTest(environment);

      double width = 0.25;

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage step = message.getFootstepDataList().add();
      step.setRobotSide(FootstepDataMessage.ROBOT_SIDE_LEFT);
      step.getLocation().set(blockDistanceFromOrigin + 0.05, width / 2, topHeight);

      drcSimulationTestHelper.publishToController(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);
   }

   @Test
   public void testWalkingOverBlock() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      double blockWidth = 0.3;
      double blockDistanceFromOrigin = 0.2;
      double topHeight = 0.1;
      SimpleBlockEnvironment environment = new SimpleBlockEnvironment(blockDistanceFromOrigin + blockWidth * 0.5, blockWidth, 0.4, topHeight);

      setupTest(environment);

      double width = 0.25;

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage step = message.getFootstepDataList().add();
      step.setRobotSide(FootstepDataMessage.ROBOT_SIDE_LEFT);
      step.getLocation().set(blockDistanceFromOrigin + 0.05, width / 2, topHeight);

      FootstepDataMessage step2 = message.getFootstepDataList().add();
      step2.setRobotSide(FootstepDataMessage.ROBOT_SIDE_RIGHT);
      step2.getLocation().set(blockDistanceFromOrigin + 0.3, -width / 2, topHeight);

      drcSimulationTestHelper.publishToController(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      assertTrue(success);
   }




   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void setupTest(CommonAvatarEnvironmentInterface environment) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // create simulation test helper
      String className = getClass().getSimpleName();
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.createSimulation(className);

      // increase ankle damping to match the real robot better
      YoDouble damping_l_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_akx");
      YoDouble damping_l_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_aky");
      YoDouble damping_r_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_akx");
      YoDouble damping_r_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_aky");
      damping_l_akx.set(1.0);
      damping_l_aky.set(1.0);
      damping_r_akx.set(1.0);
      damping_r_aky.set(1.0);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);
   }


   public class SimpleBlockEnvironment extends PlanarRegionEnvironmentInterface
   {
      public SimpleBlockEnvironment(double blockPosition, double blockWidth, double blockLength, double blockHeight)
      {
         // first ground plane
         generator.identity();
         generator.addRectangle(10.0, 10.0);

         generator.translate(blockPosition, 0.0, 0.0);

         generator.addCubeReferencedAtBottomMiddle(blockWidth, blockLength, blockHeight);

         addPlanarRegionsToTerrain(YoAppearance.Grey());
      }
   }

}
