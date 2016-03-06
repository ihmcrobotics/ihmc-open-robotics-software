package us.ihmc.darpaRoboticsChallenge.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

public abstract class EndToEndHeadTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test (timeout = 300000)
   public void testSingleWaypoint() throws Exception
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

      String headPrefix = "head";
      String orientationTrajectoryName = headPrefix + "MultipleWaypointsOrientationTrajectoryGenerator";
      String numberOfWaypointsVarName = headPrefix + "NumberOfWaypoints";
      String subTrajectoryName = headPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      double numberOfWaypoints = scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName).getValueAsDouble();
      assertEquals(2.0, numberOfWaypoints, 0.1);

      double trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qx").getValueAsDouble();
      assertEquals(desiredRandomChestOrientation.getQx(), trajOutput, epsilon);
      trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qy").getValueAsDouble();
      assertEquals(desiredRandomChestOrientation.getQy(), trajOutput, epsilon);
      trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qz").getValueAsDouble();
      assertEquals(desiredRandomChestOrientation.getQz(), trajOutput, epsilon);
      trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qs").getValueAsDouble();
      assertEquals(desiredRandomChestOrientation.getQs(), trajOutput, epsilon);
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
