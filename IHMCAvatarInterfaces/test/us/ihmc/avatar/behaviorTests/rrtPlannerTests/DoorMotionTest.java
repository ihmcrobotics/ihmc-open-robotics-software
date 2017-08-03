package us.ihmc.avatar.behaviorTests.rrtPlannerTests;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor.PushDoor;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor.PushDoorTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.TaskNode3D;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.TaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.TaskNodeTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DoorEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableDoorRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DoorMotionTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;

   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   private static RobotSide SIDE_WITH_HANDLE = RobotSide.LEFT;
   private static Vector3D STARTING_LOCATION = new Vector3D();
   private static double startYaw;

   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, isKinematicsToolboxVisualizerEnabled);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.KINEMATICS_TOOLBOX_MODULE);
   }

   public void setupCamera(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(4.0, -0.0, 1.0);
      Point3D cameraPosition = new Point3D(6.5, 2.5, 2.0);

      drcBehaviorTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (visualize)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface environment = new DoorEnvironment();

      switch (SIDE_WITH_HANDLE)
      {
      case LEFT:
         STARTING_LOCATION.set(3.4, 0.5, 0.0);
         startYaw = 1.5 * Math.PI;
         break;

      case RIGHT:
         STARTING_LOCATION.set(3.4, -0.5, 0.0);
         startYaw = 0.5 * Math.PI;
         break;

      default:
         break;
      }

      DRCStartingLocation startLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return new OffsetAndYawRobotInitialSetup(STARTING_LOCATION, startYaw);
         }
      };

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), startLocation, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }

   public ArrayList<Graphics3DObject> getXYZAxis(Pose3D pose)
   {
      double axisHeight = 0.1;
      double axisRadius = 0.01;
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject retX = new Graphics3DObject();
      Graphics3DObject retY = new Graphics3DObject();
      Graphics3DObject retZ = new Graphics3DObject();

      Point3D centerPoint = new Point3D(pose.getPosition());

      retX.translate(centerPoint);
      retY.translate(centerPoint);
      retZ.translate(centerPoint);

      RotationMatrix axisOrientation = new RotationMatrix(pose.getOrientation());

      RotationMatrix axisX = new RotationMatrix(axisOrientation);
      RotationMatrix axisY = new RotationMatrix(axisOrientation);
      RotationMatrix axisZ = new RotationMatrix(axisOrientation);

      retZ.rotate(axisZ);
      retZ.addCylinder(axisHeight, axisRadius, YoAppearance.Blue());

      axisX.appendPitchRotation(Math.PI * 0.5);
      retX.rotate(axisX);
      retX.addCylinder(axisHeight, axisRadius, YoAppearance.Red());

      axisY.appendRollRotation(-Math.PI * 0.5);
      retY.rotate(axisY);
      retY.addCylinder(axisHeight, axisRadius, YoAppearance.Green());

      ret.add(retX);
      ret.add(retY);
      ret.add(retZ);

      return ret;
   }

   @Test
   public void testForKnobGraspingPose() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      setupCamera(scs);

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      /*
       * Door define
       */
      Quaternion pushDoorOrientation = new Quaternion();
      pushDoorOrientation.appendYawRotation(startYaw);

      FramePose pushDoorFramePose = new FramePose(referenceFrames.getWorldFrame(), new Pose3D(DoorEnvironment.DEFAULT_DOOR_LOCATION, pushDoorOrientation));
      PushDoor pushDoor = new PushDoor(pushDoorFramePose, ContactableDoorRobot.DEFAULT_HANDLE_OFFSET);

      PushDoorTrajectory pushDoorTrajectory = new PushDoorTrajectory(pushDoor, 8.0, -20 * Math.PI / 180);
      pushDoorTrajectory.setRobotSideOfEndEffector(RobotSide.LEFT);

      TaskNode3D.endEffectorTrajectory = pushDoorTrajectory;

      /*
       * Door kinematics Debug.
       */
      scs.addStaticLinkGraphics(pushDoor.getGraphics());
      HandTrajectoryMessage handTrajectoryMessageOpening = pushDoorTrajectory.getEndEffectorTrajectoryMessage(referenceFrames.getMidFootZUpGroundFrame());

      System.out.println(handTrajectoryMessageOpening.getNumberOfTrajectoryPoints());
      for (int i = 0; i < handTrajectoryMessageOpening.getNumberOfTrajectoryPoints(); i++)
      {
         Point3D point = new Point3D();
         handTrajectoryMessageOpening.getTrajectoryPoints()[i].getPosition(point);

         Quaternion orientation = new Quaternion();
         handTrajectoryMessageOpening.getTrajectoryPoints()[i].getOrientation(orientation);

         Pose3D pose = new Pose3D(point, orientation);
         //         scs.addStaticLinkGraphics(getXYZAxis(pose));
      }

      /*
       * Reaching Motion
       */
      Pose3D reachingPose = pushDoorTrajectory.getEndEffectorPose(0.0);
      Point3D reachingPoint = new Point3D(reachingPose.getPosition());
      Quaternion reachingOrientation = new Quaternion(reachingPose.getOrientation());
      reachingPoint.add(new Point3D(-0.1, 0.15, 0.0));
      reachingOrientation.appendRollRotation(Math.PI * 0.5);
      HandTrajectoryMessage handTrajectoryMessageReaching = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, reachingPoint, reachingOrientation,
                                                                                      referenceFrames.getWorldFrame());

      drcBehaviorTestHelper.send(handTrajectoryMessageReaching);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(handTrajectoryMessageReaching.getTrajectoryTime());

      PrintTools.info("END");
   }

   //   @Test
   public void testForRRTPlanner() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      setupCamera(scs);

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      /*
       * Door define
       */
      Quaternion pushDoorOrientation = new Quaternion();
      pushDoorOrientation.appendYawRotation(startYaw);

      FramePose pushDoorFramePose = new FramePose(referenceFrames.getWorldFrame(), new Pose3D(DoorEnvironment.DEFAULT_DOOR_LOCATION, pushDoorOrientation));
      Vector2D doorHandle = new Vector2D(ContactableDoorRobot.DEFAULT_HANDLE_OFFSET);
      doorHandle.add(new Vector2D(-0.05, 0.0));
      PushDoor pushDoor = new PushDoor(pushDoorFramePose, doorHandle);

      PushDoorTrajectory pushDoorTrajectory = new PushDoorTrajectory(pushDoor, 8.0, -20 * Math.PI / 180);
      pushDoorTrajectory.setRobotSideOfEndEffector(RobotSide.LEFT);

      TaskNode3D.endEffectorTrajectory = pushDoorTrajectory;

      /*
       * Door kinematics Debug.
       */
      scs.addStaticLinkGraphics(pushDoor.getGraphics());
      HandTrajectoryMessage handTrajectoryMessage = pushDoorTrajectory.getEndEffectorTrajectoryMessage(referenceFrames.getMidFootZUpGroundFrame());

      System.out.println(handTrajectoryMessage.getNumberOfTrajectoryPoints());
      for (int i = 0; i < handTrajectoryMessage.getNumberOfTrajectoryPoints(); i++)
      {
         Point3D point = new Point3D();
         handTrajectoryMessage.getTrajectoryPoints()[i].getPosition(point);

         Quaternion orientation = new Quaternion();
         handTrajectoryMessage.getTrajectoryPoints()[i].getOrientation(orientation);

         Pose3D pose = new Pose3D(point, orientation);
         scs.addStaticLinkGraphics(getXYZAxis(pose));
      }

      /*
       * Initialize tester.
       */
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel(), sdfFullRobotModel);

      wbikTester.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel), sdfFullRobotModel.getRootJoint());

      TaskNode3D.nodeTester = wbikTester;
      TaskNode3D.midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();

      /*
       * Tree expanding.
       */

      double initialPelvisHeight = sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();

      TaskNode3D rootNode = new TaskNode3D(0.0, initialPelvisHeight, 0.0, 0.0);
      rootNode.setConfigurationJoints(sdfFullRobotModel);

      TaskNodeTree taskNodeTree = new TaskNodeTree(rootNode, "pelvisHeight", "chestYaw", "chestPitch");

      taskNodeTree.getTaskNodeRegion().setRandomRegion(0, 0.0, pushDoorTrajectory.getTrajectoryTime());
      taskNodeTree.getTaskNodeRegion().setRandomRegion(1, 0.75, 0.9);
      taskNodeTree.getTaskNodeRegion().setRandomRegion(2, Math.PI * (-0.2), Math.PI * (0.2));
      taskNodeTree.getTaskNodeRegion().setRandomRegion(3, Math.PI * (-0.2), Math.PI * (0.2));

      System.out.println(taskNodeTree.getTrajectoryTime());

      taskNodeTree.expandTree(2000);

      TaskNodeTreeVisualizer taskNodeTreeVisualizer = new TaskNodeTreeVisualizer(scs, taskNodeTree);
      taskNodeTreeVisualizer.visualize();

      taskNodeTree.saveNodes();

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      PrintTools.info("END");
   }

   //   @Test
   public void testForBasicPushDoorOpeningMotion() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      setupCamera(scs);

      drcBehaviorTestHelper.updateRobotModel();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      Quaternion pushDoorOrientation = new Quaternion();
      pushDoorOrientation.appendYawRotation(startYaw);

      FramePose pushDoorFramePose = new FramePose(referenceFrames.getWorldFrame(), new Pose3D(DoorEnvironment.DEFAULT_DOOR_LOCATION, pushDoorOrientation));
      Vector2D doorHandle = new Vector2D(ContactableDoorRobot.DEFAULT_HANDLE_OFFSET);
      doorHandle.add(new Vector2D(-0.05, 0.0));
      PushDoor pushDoor = new PushDoor(pushDoorFramePose, doorHandle);

      PushDoorTrajectory pushDoorTrajectory = new PushDoorTrajectory(pushDoor, 35.0, -35 * Math.PI / 180);
      pushDoorTrajectory.setRobotSideOfEndEffector(RobotSide.LEFT);

      TaskNode3D.endEffectorTrajectory = pushDoorTrajectory;

      WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      HandTrajectoryMessage handTrajectoryMessage = pushDoorTrajectory.getEndEffectorTrajectoryMessage(referenceFrames.getMidFeetZUpFrame());

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(3);
      chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());
      chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());

      chestTrajectoryMessage.setTrajectoryPoint(0, 5.0, new Quaternion(), new Vector3D(), referenceFrames.getMidFootZUpGroundFrame());
      Quaternion chestOrientation = new Quaternion();
      chestOrientation.appendYawRotation(-Math.PI * 20 / 180);
      chestOrientation.appendPitchRotation(Math.PI * 10 / 180);
      chestTrajectoryMessage.setTrajectoryPoint(1, 20.0, chestOrientation, new Vector3D(), referenceFrames.getMidFootZUpGroundFrame());

      chestOrientation = new Quaternion();
      chestOrientation.appendYawRotation(-Math.PI * 30 / 180);
      chestOrientation.appendPitchRotation(Math.PI * 10 / 180);
      chestTrajectoryMessage.setTrajectoryPoint(2, 40.0, chestOrientation, new Vector3D(), referenceFrames.getMidFootZUpGroundFrame());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(3);
      pelvisTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());
      pelvisTrajectoryMessage.getFrameInformation().setDataReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());

      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.clearSelection();
      selectionMatrix6D.selectLinearZ(true);
      pelvisTrajectoryMessage.setSelectionMatrix(selectionMatrix6D);
      pelvisTrajectoryMessage.setTrajectoryPoint(0, 5.0, new Point3D(0, 0, 0.78), new Quaternion(), new Vector3D(), new Vector3D(),
                                                 referenceFrames.getMidFootZUpGroundFrame());
      pelvisTrajectoryMessage.setTrajectoryPoint(1, 20.0, new Point3D(0, 0, 0.90), new Quaternion(), new Vector3D(), new Vector3D(),
                                                 referenceFrames.getMidFootZUpGroundFrame());
      pelvisTrajectoryMessage.setTrajectoryPoint(2, 40.0, new Point3D(0, 0, 0.85), new Quaternion(), new Vector3D(), new Vector3D(),
                                                 referenceFrames.getMidFootZUpGroundFrame());

      wholebodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
      wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);

      System.out.println(handTrajectoryMessage.getNumberOfTrajectoryPoints());
      for (int i = 0; i < handTrajectoryMessage.getNumberOfTrajectoryPoints(); i++)
      {
         Point3D point = new Point3D();
         handTrajectoryMessage.getTrajectoryPoints()[i].getPosition(point);

         Quaternion orientation = new Quaternion();
         handTrajectoryMessage.getTrajectoryPoints()[i].getOrientation(orientation);

         Pose3D pose = new Pose3D(point, orientation);
         if (i % 2 == 0)
            scs.addStaticLinkGraphics(getXYZAxis(pose));
      }

      drcBehaviorTestHelper.send(wholebodyTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(50.0);

      PrintTools.info("END");
   }

   //   @Test
   public void testForOnlyLeftArmOpeningMotion() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      setupCamera(scs);

      drcBehaviorTestHelper.updateRobotModel();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      Quaternion pushDoorOrientation = new Quaternion();
      pushDoorOrientation.appendYawRotation(startYaw);

      FramePose pushDoorFramePose = new FramePose(referenceFrames.getWorldFrame(), new Pose3D(DoorEnvironment.DEFAULT_DOOR_LOCATION, pushDoorOrientation));
      Vector2D doorHandle = new Vector2D(ContactableDoorRobot.DEFAULT_HANDLE_OFFSET);
      doorHandle.add(new Vector2D(-0.05, 0.0));
      PushDoor pushDoor = new PushDoor(pushDoorFramePose, doorHandle);

      PushDoorTrajectory pushDoorTrajectory = new PushDoorTrajectory(pushDoor, 20.0, -35 * Math.PI / 180);
      pushDoorTrajectory.setRobotSideOfEndEffector(RobotSide.LEFT);

      TaskNode3D.endEffectorTrajectory = pushDoorTrajectory;

      WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      HandTrajectoryMessage handTrajectoryMessage = pushDoorTrajectory.getEndEffectorTrajectoryMessage(referenceFrames.getMidFeetZUpFrame());

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(3);
      chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());
      chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());

      chestTrajectoryMessage.setTrajectoryPoint(0, 5.0, new Quaternion(), new Vector3D(), referenceFrames.getMidFootZUpGroundFrame());
      Quaternion chestOrientation = new Quaternion();
      chestOrientation.appendYawRotation(-Math.PI * 20 / 180);
      //      chestOrientation.appendPitchRotation(Math.PI*10/180);
      chestTrajectoryMessage.setTrajectoryPoint(1, 20.0, chestOrientation, new Vector3D(), referenceFrames.getMidFootZUpGroundFrame());

      chestOrientation = new Quaternion();
      chestOrientation.appendYawRotation(-Math.PI * 30 / 180);
      //      chestOrientation.appendPitchRotation(Math.PI*10/180);
      chestTrajectoryMessage.setTrajectoryPoint(2, 40.0, chestOrientation, new Vector3D(), referenceFrames.getMidFootZUpGroundFrame());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(3);
      pelvisTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());
      pelvisTrajectoryMessage.getFrameInformation().setDataReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());

      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.clearSelection();
      selectionMatrix6D.selectLinearZ(true);
      pelvisTrajectoryMessage.setSelectionMatrix(selectionMatrix6D);
      pelvisTrajectoryMessage.setTrajectoryPoint(0, 5.0, new Point3D(0, 0, 0.78), new Quaternion(), new Vector3D(), new Vector3D(),
                                                 referenceFrames.getMidFootZUpGroundFrame());
      pelvisTrajectoryMessage.setTrajectoryPoint(1, 20.0, new Point3D(0, 0, 0.90), new Quaternion(), new Vector3D(), new Vector3D(),
                                                 referenceFrames.getMidFootZUpGroundFrame());
      pelvisTrajectoryMessage.setTrajectoryPoint(2, 40.0, new Point3D(0, 0, 0.85), new Quaternion(), new Vector3D(), new Vector3D(),
                                                 referenceFrames.getMidFootZUpGroundFrame());

      //      wholebodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);            
      wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);

      System.out.println(handTrajectoryMessage.getNumberOfTrajectoryPoints());
      for (int i = 0; i < handTrajectoryMessage.getNumberOfTrajectoryPoints(); i++)
      {
         Point3D point = new Point3D();
         handTrajectoryMessage.getTrajectoryPoints()[i].getPosition(point);

         Quaternion orientation = new Quaternion();
         handTrajectoryMessage.getTrajectoryPoints()[i].getOrientation(orientation);

         Pose3D pose = new Pose3D(point, orientation);
         if (i % 2 == 0)
            scs.addStaticLinkGraphics(getXYZAxis(pose));
      }

      drcBehaviorTestHelper.send(wholebodyTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(50.0);

      PrintTools.info("END");
   }

}
