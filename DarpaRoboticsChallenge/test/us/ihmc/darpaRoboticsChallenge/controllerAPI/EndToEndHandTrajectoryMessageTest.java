package us.ihmc.darpaRoboticsChallenge.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndHandTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test//(timeout = 300000)
   public void testHandTrajectoryMessageWithSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage();

      Random random = new Random(564574L);
      double epsilon = 1.0e-10;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), "", selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 1.0;
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         
         OneDoFJoint[] armClone = ScrewTools.cloneOneDoFJointPath(chest, hand);
         
         ScrewTestTools.setRandomPositionsWithinJointLimits(armClone, random);

         RigidBody handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose desiredRandomHandPose = new FramePose(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(ReferenceFrame.getWorldFrame());
         
         Point3d desiredPosition = new Point3d();
         Quat4d desiredOrientation = new Quat4d();
         desiredRandomHandPose.getPose(desiredPosition, desiredOrientation);
         HandTrajectoryMessage armTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, trajectoryTime, desiredPosition, desiredOrientation);

         drcSimulationTestHelper.send(armTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         
         String handPrefix = sidePrefix + "Hand";
         String positionTrajectoryName = handPrefix + "MultipleWaypointsPositionTrajectoryGenerator";
         String orientationTrajectoryName = handPrefix + "MultipleWaypointsOrientationTrajectoryGenerator";
         String numberOfWaypointsVarName = handPrefix + "NumberOfWaypoints";
         String subTrajectoryName = handPrefix + "SubTrajectory";
         String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";
         String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

         double numberOfWaypoints = scs.getVariable(positionTrajectoryName, numberOfWaypointsVarName).getValueAsDouble();
         assertEquals(2.0, numberOfWaypoints, 0.1);
         numberOfWaypoints = scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName).getValueAsDouble();
         assertEquals(2.0, numberOfWaypoints, 0.1);

         double trajOutput = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "X").getValueAsDouble();
         assertEquals(desiredPosition.getX(), trajOutput, epsilon);
         trajOutput = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "Y").getValueAsDouble();
         assertEquals(desiredPosition.getY(), trajOutput, epsilon);
         trajOutput = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "Z").getValueAsDouble();
         assertEquals(desiredPosition.getZ(), trajOutput, epsilon);
         trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qx").getValueAsDouble();
         assertEquals(desiredOrientation.getX(), trajOutput, epsilon);
         trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qy").getValueAsDouble();
         assertEquals(desiredOrientation.getY(), trajOutput, epsilon);
         trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qz").getValueAsDouble();
         assertEquals(desiredOrientation.getZ(), trajOutput, epsilon);
         trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qs").getValueAsDouble();
         assertEquals(desiredOrientation.getW(), trajOutput, epsilon);
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
