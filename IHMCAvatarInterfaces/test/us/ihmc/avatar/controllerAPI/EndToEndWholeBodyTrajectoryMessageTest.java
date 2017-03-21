package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndWholeBodyTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 19.1)
   @Test(timeout = 95000)
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RobotSide footSide = RobotSide.LEFT;
      // First need to pick up the foot:
      RigidBody foot = fullRobotModel.getFoot(footSide);
      FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
      footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      footPoseCloseToActual.changeFrame(worldFrame);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(footSide, 0.0, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.send(footTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(success);

      // Now we can do the usual test.
      double trajectoryTime = 1.0;
      FramePose desiredFootPose = new FramePose(foot.getBodyFixedFrame());
      desiredFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredFootPose.setPosition(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.getPose(desiredPosition, desiredOrientation);
      wholeBodyTrajectoryMessage.setFootTrajectoryMessage(new FootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));

      SideDependentList<FramePose> desiredHandPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] arm = ScrewTools.createOneDoFJointPath(chest, hand);
         OneDoFJoint[] armClone = ScrewTools.cloneOneDoFJointPath(chest, hand);
         for (int i = 0; i < armClone.length; i++)
         {
            OneDoFJoint joint = armClone[i];
            joint.setQ(arm[i].getQ() + RandomNumbers.nextDouble(random, -0.2, 0.2));
         }
         RigidBody handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose desiredRandomHandPose = new FramePose(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredHandPoses.put(robotSide, desiredRandomHandPose);
         desiredPosition = new Point3D();
         desiredOrientation = new Quaternion();
         desiredRandomHandPose.getPose(desiredPosition, desiredOrientation);
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(new HandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, worldFrame, worldFrame));
      }


      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      RigidBody pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      FramePose desiredPelvisPose = new FramePose(pelvis.getBodyFixedFrame());
      desiredPelvisPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredPelvisPose.setPosition(RandomGeometry.nextPoint3D(random, 0.05, 0.03, 0.05));
      desiredPelvisPose.setZ(desiredPelvisPose.getZ() - 0.1);
      desiredPosition = new Point3D();
      desiredOrientation = new Quaternion();
      desiredPelvisPose.changeFrame(worldFrame);
      desiredPelvisPose.getPose(desiredPosition, desiredOrientation);
      wholeBodyTrajectoryMessage.setPelvisTrajectoryMessage(new PelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation));

      FrameOrientation desiredChestOrientation = new FrameOrientation(worldFrame, RandomGeometry.nextQuaternion(random, 0.5));
      desiredChestOrientation.changeFrame(worldFrame);
      desiredOrientation = new Quaternion();
      desiredChestOrientation.getQuaternion(desiredOrientation);
      wholeBodyTrajectoryMessage.setChestTrajectoryMessage(new ChestTrajectoryMessage(trajectoryTime, desiredOrientation, worldFrame, pelvisZUpFrame));

      drcSimulationTestHelper.send(wholeBodyTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         desiredHandPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      RigidBody chest = fullRobotModel.getChest();
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(worldFrame);
      EndToEndChestTrajectoryMessageTest.assertSingleWaypointExecuted(desiredChestOrientation, scs, chest);
//      EndToEndPelvisTrajectoryMessageTest.assertSingleWaypointExecuted(desiredPosition, desiredOrientation, scs);
      EndToEndFootTrajectoryMessageTest.assertSingleWaypointExecuted(footSide, desiredFootPose.getFramePointCopy().getPoint(), desiredFootPose.getFrameOrientationCopy().getQuaternion(), scs);
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = drcSimulationTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
         desiredHandPoses.get(robotSide).changeFrame(worldFrame);
         Point3D desiredHandPosition = desiredHandPoses.get(robotSide).getFramePointCopy().getPoint();
         Quaternion desiredHandOrientation = desiredHandPoses.get(robotSide).getFrameOrientationCopy().getQuaternion();

         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(handName, desiredHandPosition, desiredHandOrientation, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.9)
   @Test(timeout = 55000)
   public void testIssue47BadChestTrajectoryMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();

      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(5);
      chestTrajectoryMessage.setDataReferenceFrameId(ReferenceFrame.getWorldFrame());
      chestTrajectoryMessage.setTrajectoryReferenceFrameId(pelvisZUpFrame);
      chestTrajectoryMessage.setTrajectoryPoint(0, 0.00, new Quaternion(), new Vector3D(), ReferenceFrame.getWorldFrame());
      chestTrajectoryMessage.setTrajectoryPoint(1, 0.10, new Quaternion(), new Vector3D(), ReferenceFrame.getWorldFrame());
      chestTrajectoryMessage.setTrajectoryPoint(2, 0.20, new Quaternion(), new Vector3D(), ReferenceFrame.getWorldFrame());
      chestTrajectoryMessage.setTrajectoryPoint(3, 0.10, new Quaternion(), new Vector3D(), ReferenceFrame.getWorldFrame());
      chestTrajectoryMessage.setTrajectoryPoint(4, 0.00, new Quaternion(), new Vector3D(), ReferenceFrame.getWorldFrame());
      
      
      
      
      wholeBodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      drcSimulationTestHelper.send(wholeBodyTrajectoryMessage);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.8)
   @Test(timeout = 54000)
   public void testIssue47BadPelvisTrajectoryMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);

      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(5);
      pelvisTrajectoryMessage.setTrajectoryPoint(0, 0.00, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D());
      pelvisTrajectoryMessage.setTrajectoryPoint(1, 0.10, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D());
      pelvisTrajectoryMessage.setTrajectoryPoint(2, 0.20, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D());
      pelvisTrajectoryMessage.setTrajectoryPoint(3, 0.10, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D());
      pelvisTrajectoryMessage.setTrajectoryPoint(4, 0.00, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D());
      wholeBodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
      drcSimulationTestHelper.send(wholeBodyTrajectoryMessage);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
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
