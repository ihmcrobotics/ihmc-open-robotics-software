package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.dataProcessors.RobotAllJointsDataChecker;
import us.ihmc.simulationconstructionset.util.dataProcessors.ValueDataCheckerParameters;
import us.ihmc.simulationconstructionset.util.dataProcessors.YoVariableValueDataChecker;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifier;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifierWithStackTracing;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.QuarantinedTest;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.math.trajectories.TrajectoryType;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

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


   @BambooAnnotations.EstimatedDuration(duration = 30.4)
   @Test(timeout = 151825)
   public void testWalkingWithHighSteps() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOverSmallPlatformTest", "", selectedLocation, simulationTestingParameters,
              getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();


      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);

      FootstepDataList footstepDataList = createFootstepsWithHighSwing();
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(25.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(2.0, 0.0, 0.75);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
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

      BambooTools.reportTestFinishedMessage();
   }

   private FootstepDataList createFootstepsWithHighSwing()
   {
      double swingHeight = 0.5;
      double swingTime = 1.5;
      double transferTime = 1.0;
      Quat4d orientation = new Quat4d();
      Vector3d verticalVector = new Vector3d(0.0, 0.0, 1.0);
      FootstepDataList footstepDataList = new FootstepDataList(swingTime, transferTime);
      RotationFunctions.getQuaternionFromYawAndZNormal(0.0 / 4.0 * Math.PI, verticalVector, orientation);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(0.0, .15, 0.0), new Quat4d(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(0.0, -.15, 0.0), new Quat4d(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(0.4, .15, 0.0), new Quat4d(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(0.8, -.15, 0.0), new Quat4d(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(1.2, .15, 0.0), new Quat4d(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(1.6, -.15, 0.0), new Quat4d(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(2.0, .15, 0.0), new Quat4d(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(2.0, -.15, 0.0), new Quat4d(orientation), null, TrajectoryType.OBSTACLE_CLEARANCE, swingHeight));


      return footstepDataList;
   }
}
