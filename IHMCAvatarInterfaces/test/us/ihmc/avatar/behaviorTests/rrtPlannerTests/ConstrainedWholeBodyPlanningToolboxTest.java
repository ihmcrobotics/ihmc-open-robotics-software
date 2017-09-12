package us.ihmc.avatar.behaviorTests.rrtPlannerTests;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.ConstrainedWholeBodyPlanningToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PlanConstrainedWholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.RobotKinematicsConfiguration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.DrawingTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
import us.ihmc.tools.thread.ThreadTools;

public abstract class ConstrainedWholeBodyPlanningToolboxTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   private static boolean visulaizerOn = true;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private ConstrainedWholeBodyPlanningToolboxModule cwbPlanningToolboxModule;

   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;

   private void setupCWBPlanningToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false);
      cwbPlanningToolboxModule = new ConstrainedWholeBodyPlanningToolboxModule(robotModel, fullRobotModel, null, visulaizerOn);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
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

      if (cwbPlanningToolboxModule != null)
      {
         cwbPlanningToolboxModule.destroy();
         cwbPlanningToolboxModule = null;
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

      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupCWBPlanningToolboxModule();
   }

   //   @Test
   public void testForBehavior() throws SimulationExceededMaximumTimeException, IOException
   {

   }

   @Test
   public void testForToolbox() throws SimulationExceededMaximumTimeException, IOException
   {
      if (visulaizerOn)
         ThreadTools.sleep(1000);

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      /*
       * reaching initial configuration
       */
      Quaternion initialOrientation = new Quaternion();
      initialOrientation.appendRollRotation(Math.PI * 0.5);
      initialOrientation.appendYawRotation(Math.PI * 0.5);
      initialOrientation.appendPitchRotation(-Math.PI * 0.4);
      HandTrajectoryMessage lhandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 2.0, new Point3D(0.6, 0.35, 1.0), initialOrientation,
                                                                               referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(lhandTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());

      initialOrientation = new Quaternion();
      initialOrientation.appendPitchRotation(Math.PI * 0.4);
      HandTrajectoryMessage rhandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2.0, new Point3D(-0.1, -0.5, 0.7), initialOrientation,
                                                                               referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(rhandTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      /*
       * run toolbox.
       */
      System.out.println("Start");

      ToolboxStateMessage toolboxMessage;

      toolboxMessage = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      toolboxMessage.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
      toolboxCommunicator.send(toolboxMessage);

      /*
       * constrained end effector trajectory.
       */
      System.out.println("Send packet " + drcBehaviorTestHelper.getYoTime());
      ConstrainedEndEffectorTrajectory endeffectorTrajectory = new DrawingTrajectory(10.0);

      ConstrainedWholeBodyPlanningRequestPacket packet = new ConstrainedWholeBodyPlanningRequestPacket();

      PlanConstrainedWholeBodyTrajectoryBehavior.constrainedEndEffectorTrajectory = endeffectorTrajectory;
      packet.setNumberOfFindInitialGuess(200);
      packet.setNumberOfExpanding(600);
      packet.setInitialRobotConfigration(sdfFullRobotModel);

      packet.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);

      toolboxCommunicator.send(packet);
      System.out.println("Send packet done" + drcBehaviorTestHelper.getYoTime());

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(10.0);

      /*
       * test motion.
       */

      System.out.println("End");

   }

   //      @Test
   public void testForInverseKinematicsToolbox() throws SimulationExceededMaximumTimeException, IOException
   {
      if (visulaizerOn)
         ThreadTools.sleep(6000);

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

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
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(15.0);

      System.out.println("End");
   }

   //   @Test
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
      initialOrientation.appendRollRotation(Math.PI * 0.5);
      initialOrientation.appendYawRotation(Math.PI * 0.5);
      initialOrientation.appendPitchRotation(-Math.PI * 0.3);
      HandTrajectoryMessage lhandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 2.0, new Point3D(0.6, 0.35, 1.0), initialOrientation,
                                                                               referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(lhandTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());

      initialOrientation = new Quaternion();
      initialOrientation.appendPitchRotation(Math.PI * 0.3);
      HandTrajectoryMessage rhandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2.0, new Point3D(-0.1, -0.45, 0.8), initialOrientation,
                                                                               referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(rhandTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      /*
       * constrained end effector trajectory (WorldFrame).
       */
      DrawingTrajectory endeffectorTrajectory = new DrawingTrajectory(10.0);

      /*
       * solver.
       */
      WheneverWholeBodyKinematicsSolver kinematicsSolver = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      kinematicsSolver.updateRobotConfigurationData(new RobotKinematicsConfiguration(sdfFullRobotModel));

      kinematicsSolver.initialize();
      kinematicsSolver.holdCurrentTrajectoryMessages();

      Point3D desiredPoint = new Point3D(0.5, 0.35, 1.5);
      Quaternion desiredOrientation = new Quaternion();
      Pose3D desiredPose = new Pose3D(desiredPoint, desiredOrientation);

      kinematicsSolver.setDesiredHandPose(RobotSide.LEFT, desiredPose);

      kinematicsSolver.putTrajectoryMessages();
      PrintTools.info("" + kinematicsSolver.solve());

      showUpFullRobotModelWithConfiguration(sdfFullRobotModel);

      showUpFullRobotModelWithConfiguration(kinematicsSolver.getDesiredFullRobotModel());

      scs.addStaticLinkGraphics(getXYZAxis(desiredPose));

      System.out.println("End");
   }

   @Test
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

      //      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(0.0, new ConfigurationSpace())));
      //
      //      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(3.0, new ConfigurationSpace())));
      //
      //      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(6.0, new ConfigurationSpace())));
      //
      //      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(9.0, new ConfigurationSpace())));
      //
      //      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(10.0, new ConfigurationSpace())));

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("End");
   }

   // @Test
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

      System.out.println("End");
   }

   // @Test
   public void testForAtlasKinematicsConfiguration() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      RobotKinematicsConfiguration configuration = new RobotKinematicsConfiguration(sdfFullRobotModel);

      System.out.println("End");
   }

   // @Test
   public void testForToolboxMessage() throws SimulationExceededMaximumTimeException, IOException
   {
      ThreadTools.sleep(10000);

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      System.out.println("Send wake up " + drcBehaviorTestHelper.getYoTime());

      ToolboxStateMessage toolboxMessage;
      toolboxMessage = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      toolboxMessage.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
      toolboxCommunicator.send(toolboxMessage);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("Send input " + drcBehaviorTestHelper.getYoTime());

      ConstrainedWholeBodyPlanningRequestPacket requestPacket = new ConstrainedWholeBodyPlanningRequestPacket();
      toolboxCommunicator.send(requestPacket);

      System.out.println("Send input Done " + drcBehaviorTestHelper.getYoTime());

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("End");
   }

}
