package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.dataProcessors.RobotAllJointsDataChecker;
import us.ihmc.simulationConstructionSetTools.util.dataProcessors.ValueDataCheckerParameters;
import us.ihmc.simulationConstructionSetTools.util.dataProcessors.YoVariableValueDataChecker;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Created by agrabertilton on 4/15/15.
 */
public abstract class DRCHighSwingTest implements MultiRobotTestInterface
{
   protected SimulationTestingParameters simulationTestingParameters;
   protected DRCSimulationTestHelper drcSimulationTestHelper;

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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   public void testWalkingWithHighSteps() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGroundEnvironment);
      drcSimulationTestHelper.createSimulation("DRCWalkingOverSmallPlatformTest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();


      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);

      FootstepDataListMessage footstepDataList = createFootstepsWithHighSwing();
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(23.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(2.0, 0.0, 0.75);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      RobotAllJointsDataChecker checker = new RobotAllJointsDataChecker(simulationConstructionSet, drcSimulationTestHelper.getRobot());
      ArrayList<YoVariable<?>> poseVariables = simulationConstructionSet.getVariablesThatContain("adjustedDesiredPose");

      ValueDataCheckerParameters checkerParameters = new ValueDataCheckerParameters();
      checkerParameters.setMaximumDerivative(0.1);
      ArrayList<YoVariableValueDataChecker> dataCheckers = new ArrayList<>();
//      for (YoVariable poseVariable: poseVariables){
//         dataCheckers.add(new YoVariableValueDataChecker(simulationConstructionSet, poseVariable,));
//      }
//
//      YoVariableValueDataChecker translationCheckerX = new YoVariableValueDataChecker()
      simulationConstructionSet.applyDataProcessingFunction(checker);

      assertFalse(checker.hasMaxValueExeededAnyJoint());
      assertFalse(checker.hasMinValueExeededAnyJoint());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FootstepDataListMessage createFootstepsWithHighSwing()
   {
      double swingHeight = 0.5;
      double swingTime = 1.5;
      double transferTime = 1.0;
      Quaternion orientation = new Quaternion();
      Vector3D verticalVector = new Vector3D(0.0, 0.0, 1.0);
      FootstepDataListMessage footstepDataList = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      RotationTools.computeQuaternionFromYawAndZNormal(0.0 / 4.0 * Math.PI, verticalVector, orientation);
      footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(0.0, .15, 0.0), new Quaternion(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(0.0, -.15, 0.0), new Quaternion(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(0.4, .15, 0.0), new Quaternion(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(0.8, -.15, 0.0), new Quaternion(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(1.2, .15, 0.0), new Quaternion(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(1.6, -.15, 0.0), new Quaternion(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(2.0, .15, 0.0), new Quaternion(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(2.0, -.15, 0.0), new Quaternion(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));


      return footstepDataList;
   }
}
