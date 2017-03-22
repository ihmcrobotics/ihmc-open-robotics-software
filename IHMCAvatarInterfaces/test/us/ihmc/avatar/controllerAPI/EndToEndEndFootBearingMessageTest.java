package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndEndFootBearingMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @SuppressWarnings("unchecked")
   @ContinuousIntegrationTest(estimatedDuration = 41.3)
   @Test(timeout = 210000)
   public void testSwitchFootToLoadBearing() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         FramePose footPoseCloseToActual = new FramePose(fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG));
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.05);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

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

         WalkingStateEnum walkingState = ((EnumYoVariable<WalkingStateEnum>)scs.getVariable("WalkingHighLevelHumanoidController", "walkingState")).getEnumValue();
         assertEquals(WalkingStateEnum.STANDING, walkingState);
         ConstraintType footState = ((EnumYoVariable<ConstraintType>)scs.getVariable(sidePrefix + "FootControlModule", sidePrefix + "FootState")).getEnumValue();
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
