package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndHeadTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 17.9)
   @Test(timeout = 89000)
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBody head = fullRobotModel.getHead();
      RigidBody chest = fullRobotModel.getChest();

      OneDoFJoint[] neckClone = ScrewTools.cloneOneDoFJointPath(chest, head);

      ScrewTestTools.setRandomPositionsWithinJointLimits(neckClone, random);

      RigidBody headClone = neckClone[neckClone.length - 1].getSuccessor();
      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(headClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quat4d desiredOrientation = new Quat4d();
      desiredRandomChestOrientation.getQuaternion(desiredOrientation);
      HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(trajectoryTime, desiredOrientation);
      drcSimulationTestHelper.send(headTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);
      desiredRandomChestOrientation.changeFrame(chest.getBodyFixedFrame());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      
      assertSingleWaypointExecuted(desiredRandomChestOrientation.getQuaternion(), scs);
   }

   @SuppressWarnings("unchecked")
   public static HeadControlMode findControllerState(SimulationConstructionSet scs)
   {
      String headOrientatManagerName = HeadOrientationManager.class.getSimpleName();
      String headControlStateName = "headControlState";
      return ((EnumYoVariable<HeadControlMode>) scs.getVariable(headOrientatManagerName, headControlStateName)).getEnumValue();
   }

   public static double findControllerSwitchTime(SimulationConstructionSet scs)
   {
      String headOrientatManagerName = HeadOrientationManager.class.getSimpleName();
      String headControlStateName = "headControlState";
      return scs.getVariable(headOrientatManagerName, headControlStateName + "SwitchTime").getValueAsDouble();
   }

   public static Quat4d findControllerDesiredOrientation(SimulationConstructionSet scs)
   {
      String headPrefix = "head";
      String subTrajectoryName = headPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      Quat4d desiredOrientation = new Quat4d();
      desiredOrientation.setX(scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qx").getValueAsDouble());
      desiredOrientation.setY(scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qy").getValueAsDouble());
      desiredOrientation.setZ(scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qz").getValueAsDouble());
      desiredOrientation.setW(scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qs").getValueAsDouble());
      return desiredOrientation;
   }

   public static int findNumberOfWaypoints(SimulationConstructionSet scs)
   {
      String headPrefix = "head";
      String numberOfWaypointsVarName = headPrefix + "NumberOfWaypoints";
      String orientationTrajectoryName = headPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      return ((IntegerYoVariable) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static void assertSingleWaypointExecuted(Quat4d desiredOrientation, SimulationConstructionSet scs)
   {
      assertEquals(2, findNumberOfWaypoints(scs));

      Quat4d controllerDesiredOrientation = findControllerDesiredOrientation(scs);
      assertEquals(desiredOrientation.getX(), controllerDesiredOrientation.getX(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getY(), controllerDesiredOrientation.getY(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getZ(), controllerDesiredOrientation.getZ(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getW(), controllerDesiredOrientation.getW(), EPSILON_FOR_DESIREDS);
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
