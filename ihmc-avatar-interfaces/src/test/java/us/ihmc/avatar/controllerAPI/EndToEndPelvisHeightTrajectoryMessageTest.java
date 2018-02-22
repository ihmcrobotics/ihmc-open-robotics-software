package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class EndToEndPelvisHeightTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private static final boolean DEBUG = false;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);
      double epsilon = 1.0e-4;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePoint3D desiredRandomPelvisPosition = getRandomPelvisPosition(random, pelvis);
      Point3D desiredPosition = new Point3D(desiredRandomPelvisPosition);

      if (DEBUG)
      {
         System.out.println(desiredPosition);
      }

      desiredRandomPelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPosition.set(desiredRandomPelvisPosition);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
      }

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(trajectoryTime, desiredPosition.getZ());

      drcSimulationTestHelper.send(pelvisHeightTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      // Hard to figure out how to verify the desired there
//      trajOutput = scs.getVariable("pelvisHeightOffsetSubTrajectoryCubicPolynomialTrajectoryGenerator", "pelvisHeightOffsetSubTrajectoryCurrentValue").getValueAsDouble();
//      assertEquals(desiredPosition.getZ(), trajOutput, epsilon);
      // Ending up doing a rough check on the actual height
      double pelvisHeight = scs.getVariable("PelvisLinearStateUpdater", "estimatedRootJointPositionZ").getValueAsDouble();
      assertEquals(desiredPosition.getZ(), pelvisHeight, 0.01);
      
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   protected FramePoint3D getRandomPelvisPosition(Random random, RigidBody pelvis)
   {
      FramePoint3D desiredRandomPelvisPosition = new FramePoint3D(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPosition.set(RandomGeometry.nextPoint3D(random, 0.10, 0.20, 0.05));
      desiredRandomPelvisPosition.setZ(desiredRandomPelvisPosition.getZ() - 0.1);
      return desiredRandomPelvisPosition;
   }

   public void testSingleWaypointInUserMode() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePoint3D desiredRandomPelvisPosition = getRandomPelvisPosition(random, pelvis);
      Point3D desiredPosition = new Point3D(desiredRandomPelvisPosition);

      System.out.println(desiredPosition);

      desiredRandomPelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPosition.set(desiredRandomPelvisPosition);
      System.out.println(desiredPosition);

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(trajectoryTime, desiredPosition.getZ());

      pelvisHeightTrajectoryMessage.setEnableUserPelvisControl(true);
      drcSimulationTestHelper.send(pelvisHeightTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      double pelvisHeight = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getTranslationZ();
      assertEquals(desiredPosition.getZ(), pelvisHeight, 0.01);
      
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public void testSingleWaypointThenManualChange() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      String namespace = LookAheadCoMHeightTrajectoryGenerator.class.getSimpleName();
      YoDouble offsetHeight = (YoDouble) scs.getVariable(namespace, "offsetHeightAboveGround");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      RigidBody pelvis = fullRobotModel.getPelvis();
      MovingReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvisFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      double initialPelvisHeight = pelvisPosition.getZ();

      Random random = new Random(4929L);
      for (int i = 0; i < 5; i++)
      {
         double offset1 = 0.06 * 2.0 * (random.nextDouble() - 0.5);
         double offset2 = 0.06 * 2.0 * (random.nextDouble() - 0.5);

         // Move pelvis using YoVariable
         offsetHeight.set(offset1);
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5));
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         Assert.assertEquals(initialPelvisHeight + offset1, pelvisPosition.getZ(), 0.01);

         // Move pelvis through message
         double desiredHeight = initialPelvisHeight + offset2;
         PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(0.5, desiredHeight);
         drcSimulationTestHelper.send(pelvisHeightTrajectoryMessage);
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5));
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         Assert.assertEquals(desiredHeight, pelvisPosition.getZ(), 0.01);
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
