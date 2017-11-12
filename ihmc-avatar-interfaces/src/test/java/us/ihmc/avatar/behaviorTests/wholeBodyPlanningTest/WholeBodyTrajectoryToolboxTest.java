package us.ihmc.avatar.behaviorTests.wholeBodyPlanningTest;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.WholeBodyTrajectoryToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WayPointsPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.DrawingTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.WayPointsTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class WholeBodyTrajectoryToolboxTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   private static boolean visulaizerOn = true;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private WholeBodyTrajectoryToolboxModule wholeBodyTrajectoryToolboxModule;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator kinematicsToolboxCommunicator;
   private PacketCommunicator wholeBodyTrajectoryToolboxCommunicator;

   private void setupWholeBodyTrajectoryToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false);
      wholeBodyTrajectoryToolboxModule = new WholeBodyTrajectoryToolboxModule(robotModel, fullRobotModel, null, visulaizerOn);
      kinematicsToolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT,
                                                                                             PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      wholeBodyTrajectoryToolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_PORT,
                                                                                                      PacketDestination.WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE);
   }

   private void showUpFullRobotModelWithConfiguration(FullHumanoidRobotModel createdFullRobotModel) throws SimulationExceededMaximumTimeException
   {
      for (int i = 0; i < createdFullRobotModel.getOneDoFJoints().length; i++)
      {
         double jointPosition = createdFullRobotModel.getOneDoFJoints()[i].getQ();
         Joint scsJoint = drcBehaviorTestHelper.getRobot().getJoint(createdFullRobotModel.getOneDoFJoints()[i].getName());

         if (scsJoint instanceof PinJoint)
         {
            PinJoint pinJoint = (PinJoint) scsJoint;
            pinJoint.setQ(jointPosition);
         }
         else
         {
            PrintTools.info(createdFullRobotModel.getOneDoFJoints()[i].getName() + " was not a PinJoint.");
         }
      }

      FloatingJoint scsRootJoint = drcBehaviorTestHelper.getRobot().getRootJoint();
      scsRootJoint.setQuaternion(new Quaternion(createdFullRobotModel.getRootJoint().getRotationForReading()));
      scsRootJoint.setPosition(new Point3D(createdFullRobotModel.getRootJoint().getTranslationForReading()));

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.005);
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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      if (wholeBodyTrajectoryToolboxModule != null)
      {
         wholeBodyTrajectoryToolboxModule.destroy();
         wholeBodyTrajectoryToolboxModule = null;
      }

      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (kinematicsToolboxCommunicator != null)
      {
         kinematicsToolboxCommunicator.closeConnection();
         kinematicsToolboxCommunicator = null;
      }

      if (wholeBodyTrajectoryToolboxCommunicator != null)
      {
         wholeBodyTrajectoryToolboxCommunicator.closeConnection();
         wholeBodyTrajectoryToolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupWholeBodyTrajectoryToolboxModule();
   }

   public void testForInverseKinematicsToolbox() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      RobotSide robotSide = RobotSide.RIGHT;

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                                     drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrame = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(robotSide);

      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      ReferenceFrame chestControlFrame = chest.getBodyFixedFrame();
      FrameOrientation initialChestOrientation = new FrameOrientation(chestControlFrame);
      initialChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      ReferenceFrame pelvisControlFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FrameOrientation initialPelvisOrientation = new FrameOrientation(pelvisControlFrame);
      initialPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose desiredHandPose = new FramePose(handControlFrame);
      desiredHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPose.prependTranslation(0.20, 0.0, 0.0); // when prepend in z direction, left hand movement is strange.      
      ik.setTrajectoryTime(3.0);
      ik.setDesiredHandPose(robotSide, desiredHandPose);
      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();
      ik.holdCurrentPelvisHeight();

      drcBehaviorTestHelper.updateRobotModel();
      FramePose desiredHandPoseCopy = new FramePose(desiredHandPose);
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      desiredHandPoseCopy.changeFrame(chestFrame);

      drcBehaviorTestHelper.dispatchBehavior(ik);
      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(8.0));

      System.out.println("End");
   }

   public void testForTrajSolver() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      /*
       * EE Traj
       */
      Pose3D[] poseOfWayPoints = new Pose3D[4];
      Quaternion wayPointOrientation;

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.2);
      poseOfWayPoints[0] = new Pose3D(new Point3D(0.6, 0.4, 1.0), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(-Math.PI * 0.2);
      wayPointOrientation.appendYawRotation(-Math.PI * 0.3);
      poseOfWayPoints[1] = new Pose3D(new Point3D(0.6, 0.1, 1.5), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendYawRotation(-Math.PI * 0.2);
      poseOfWayPoints[2] = new Pose3D(new Point3D(0.5, -0.2, 1.0), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.2);
      poseOfWayPoints[3] = new Pose3D(new Point3D(0.6, 0.4, 0.8), wayPointOrientation);

      WayPointsPacket wallPosePacket = new WayPointsPacket();
      wallPosePacket.setRobotSide(RobotSide.LEFT);
      wallPosePacket.setWayPoints(poseOfWayPoints, 10.0);

      ConstrainedEndEffectorTrajectory endeffectorTrajectory = new WayPointsTrajectory(wallPosePacket);

      /*
       * solver.
       */
      WheneverWholeBodyKinematicsSolver kinematicsSolver = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      kinematicsSolver.updateRobotConfigurationData(new KinematicsToolboxOutputStatus(sdfFullRobotModel.getRootJoint(),
                                                                                      FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel),
                                                                                      false));

      kinematicsSolver.initialize();
      kinematicsSolver.holdCurrentTrajectoryMessages();

      for (RobotSide robotSide : RobotSide.values)
      {
         Pose3D desiredPose = endeffectorTrajectory.getEndEffectorPose(0.0, robotSide, new ConfigurationSpace());

         desiredPose.appendTranslation(-0.0, 0.0, 0.0);

         kinematicsSolver.setDesiredHandPose(robotSide, desiredPose);

         scs.addStaticLinkGraphics(getXYZAxis(desiredPose));
      }

      Quaternion desiredChestOrientation = new Quaternion();
      kinematicsSolver.setDesiredChestOrientation(desiredChestOrientation);

      kinematicsSolver.setDesiredPelvisHeight(0.8);

      kinematicsSolver.putTrajectoryMessages();

      /*
       * result
       */
      PrintTools.info("" + kinematicsSolver.solve());

      showUpFullRobotModelWithConfiguration(kinematicsSolver.getDesiredFullRobotModel());
      //
      // scs.addStaticLinkGraphics(getXYZAxis(desiredPose));

      System.out.println("End");
   }

   public void testForSolver() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      /*
       * reaching initial configuration
       */

      Quaternion initialOrientation = new Quaternion();
      initialOrientation.appendPitchRotation(Math.PI * 0.3);
      HandTrajectoryMessage lhandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 1.0, new Point3D(0.0, 0.4, 0.75), initialOrientation,
                                                                               referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(lhandTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());

      initialOrientation = new Quaternion();
      initialOrientation.appendPitchRotation(Math.PI * 0.3);
      HandTrajectoryMessage rhandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 1.0, new Point3D(-0.0, -0.4, 0.75), initialOrientation,
                                                                               referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(rhandTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      /*
       * solver.
       */
      WheneverWholeBodyKinematicsSolver kinematicsSolver = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      kinematicsSolver.updateRobotConfigurationData(new KinematicsToolboxOutputStatus(sdfFullRobotModel.getRootJoint(),
                                                                                      FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel),
                                                                                      false));

      kinematicsSolver.initialize();
      kinematicsSolver.holdCurrentTrajectoryMessages();

      Point3D desiredPoint = new Point3D(0.7, 0.35, 1.0);
      Quaternion desiredOrientation = new Quaternion();
      Pose3D desiredPose = new Pose3D(desiredPoint, desiredOrientation);

      kinematicsSolver.setDesiredHandPose(RobotSide.LEFT, desiredPose);

      kinematicsSolver.putTrajectoryMessages();
      PrintTools.info("" + kinematicsSolver.solve());

      kinematicsSolver.updateRobotConfigurationData(new KinematicsToolboxOutputStatus(sdfFullRobotModel.getRootJoint(),
                                                                                      FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel),
                                                                                      false));

      kinematicsSolver.initialize();
      kinematicsSolver.holdCurrentTrajectoryMessages();

      desiredPoint = new Point3D(0.6, 0.35, 1.0);
      desiredOrientation = new Quaternion();
      desiredPose = new Pose3D(desiredPoint, desiredOrientation);

      kinematicsSolver.setDesiredHandPose(RobotSide.LEFT, desiredPose);

      kinematicsSolver.putTrajectoryMessages();
      PrintTools.info("" + kinematicsSolver.solve());

      // showUpFullRobotModelWithConfiguration(sdfFullRobotModel);
      // showUpFullRobotModelWithConfiguration(kinematicsSolver.getDesiredFullRobotModel());
      // scs.addStaticLinkGraphics(getXYZAxis(desiredPose));

      System.out.println("End");
   }

   public void testForConstrainedTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      /*
       * constrained end effector trajectory
       */

      DrawingTrajectory endeffectorTrajectory = new DrawingTrajectory(10.0);

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(0.0, RobotSide.LEFT, new ConfigurationSpace())));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(3.0, RobotSide.LEFT, new ConfigurationSpace())));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(6.0, RobotSide.LEFT, new ConfigurationSpace())));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(9.0, RobotSide.LEFT, new ConfigurationSpace())));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(10.0, RobotSide.LEFT, new ConfigurationSpace())));

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("End");
   }

   public void testForConfigurationSpace() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      Point3D translation;
      Quaternion orientation;
      Pose3D pose3D;

      ConfigurationBuildOrder configurationBuildOrder;
      ConfigurationSpace configurationSpace;

      configurationBuildOrder = new ConfigurationBuildOrder(ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z,
                                                            ConfigurationSpaceName.YAW, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL);

      configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(0.5, 0.0, 1.0);
      configurationSpace.setRotation(Math.PI / 180 * 30, 0, 0);

      translation = new Point3D(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getTranslationVector());
      orientation = new Quaternion(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getRotationMatrix());
      pose3D = new Pose3D(translation, orientation);

      scs.addStaticLinkGraphics(getXYZAxis(pose3D));

      configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(1.0, 0.0, 1.0);
      configurationSpace.setRotation(Math.PI / 180 * 30, Math.PI / 180 * 30, 0);

      translation = new Point3D(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getTranslationVector());
      orientation = new Quaternion(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getRotationMatrix());
      pose3D = new Pose3D(translation, orientation);

      scs.addStaticLinkGraphics(getXYZAxis(pose3D));

      configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(1.5, 0.0, 1.0);
      configurationSpace.setRotation(Math.PI / 180 * 30, 0, Math.PI / 180 * 30);

      translation = new Point3D(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getTranslationVector());
      orientation = new Quaternion(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getRotationMatrix());
      pose3D = new Pose3D(translation, orientation);

      scs.addStaticLinkGraphics(getXYZAxis(pose3D));

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      System.out.println("End");
   }

}