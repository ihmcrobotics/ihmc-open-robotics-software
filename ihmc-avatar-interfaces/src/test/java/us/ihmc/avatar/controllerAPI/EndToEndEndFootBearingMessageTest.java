package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class EndToEndEndFootBearingMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @SuppressWarnings("unchecked")
   public void testSwitchFootToLoadBearing() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         FramePose3D footPoseCloseToActual = new FramePose3D(fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG));
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.05);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.get(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         FootLoadBearingMessage footLoadBearingMessage = new FootLoadBearingMessage(robotSide, LoadBearingRequest.LOAD);

         drcSimulationTestHelper.send(footLoadBearingMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.5);
         assertTrue(success);

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         WalkingStateEnum walkingState = ((YoEnum<WalkingStateEnum>)scs.getVariable("WalkingHighLevelHumanoidController", "walkingState")).getEnumValue();
         assertEquals(WalkingStateEnum.STANDING, walkingState);
         ConstraintType footState = ((YoEnum<ConstraintType>)scs.getVariable(sidePrefix + "FootControlModule", sidePrefix + "FootState")).getEnumValue();
         assertEquals(ConstraintType.FULL, footState);
      }
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
