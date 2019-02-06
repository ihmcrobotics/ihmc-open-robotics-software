package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import static us.ihmc.robotics.Assert.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage;
import controller_msgs.msg.dds.KinematicsPlanningToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarHumanoidKinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class AvatarKinematicsPlanningToolboxControllerTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();
   static
   {
      simulationTestingParameters.setKeepSCSUp(false);
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private CommandInputManager commandInputManager;
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private KinematicsPlanningToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;

   private SimulationConstructionSet scs;
   private BlockingSimulationRunner blockingSimulationRunner;

   private HumanoidFloatingRootJointRobot robot;
   private HumanoidFloatingRootJointRobot ghost;
   private RobotController toolboxUpdater;

   /**
    * Returns a separate instance of the robot model that will be modified in this test to create a
    * ghost robot.
    */
   public abstract DRCRobotModel getGhostRobotModel();

   @BeforeEach
   public void setup()
   {
      mainRegistry = new YoVariableRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();

      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
      commandInputManager = new CommandInputManager(KinematicsPlanningToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsPlanningToolboxCommandConverter(desiredFullRobotModel));

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsPlanningToolboxModule.supportedStatus());

      toolboxController = new KinematicsPlanningToolboxController(robotModel, desiredFullRobotModel, commandInputManager, statusOutputManager,
                                                                  yoGraphicsListRegistry, mainRegistry);

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName("Ghost");
      KinematicsToolboxControllerTest.recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), ghostApperance);
      ghost = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      ghost.setDynamic(false);
      ghost.setGravity(0);
      hideGhost();

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot[] {robot, ghost}, simulationTestingParameters);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setCameraFix(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
         scs.startOnAThread();
         blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);
      }
   }

   private void hideGhost()
   {
      ghost.setPositionInWorld(new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
   }

   private void snapGhostToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      new JointAnglesWriter(ghost, fullHumanoidRobotModel).updateRobotConfigurationBasedOnFullRobotModel();
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (mainRegistry != null)
      {
         mainRegistry.closeAndDispose();
         mainRegistry = null;
      }

      initializationSucceeded = null;

      yoGraphicsListRegistry = null;

      commandInputManager = null;

      toolboxController = null;

      robot = null;
      toolboxUpdater = null;
      blockingSimulationRunner = null;

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }

      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testDualHandTrajectory() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      RobotSide robotSide = RobotSide.LEFT;
      RigidBodyBasics endEffector = initialFullRobotModel.getHand(robotSide);
      double trajectoryTime = 5.0;
      int numberOfKeyFrames = 10;
      FramePose3D initialPose = new FramePose3D(endEffector.getBodyFixedFrame());
      FramePose3D desiredPose = new FramePose3D(endEffector.getBodyFixedFrame(), new Point3D(0.1, 0.1, 0.6), new AxisAngle(1.0, 0.0, 0.0, 0.5 * Math.PI));
      initialPose.changeFrame(worldFrame);
      desiredPose.changeFrame(worldFrame);

      TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
      List<Pose3DReadOnly> keyFramePoses = new ArrayList<Pose3DReadOnly>();
      List<Point3DReadOnly> desiredCOMPoints = new ArrayList<Point3DReadOnly>();

      for (int i = 0; i < numberOfKeyFrames; i++)
      {
         double alpha = (i + 1) / (double) (numberOfKeyFrames);
         keyFrameTimes.add(alpha * trajectoryTime);
         Pose3D pose = new Pose3D(initialPose);
         pose.interpolate(desiredPose, alpha);
         keyFramePoses.add(pose);
         desiredCOMPoints.add(new Point3D());
         if (visualize)
            scs.addStaticLinkGraphics(createEndEffectorKeyFrameVisualization(pose));
      }

      KinematicsPlanningToolboxRigidBodyMessage endEffectorMessage = HumanoidMessageTools.createKinematicsPlanningToolboxRigidBodyMessage(endEffector,
                                                                                                                                          keyFrameTimes,
                                                                                                                                          keyFramePoses);

      endEffectorMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      endEffectorMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));

      KinematicsPlanningToolboxRigidBodyMessage holdAnotherHandMessage = KinematicsPlanningToolboxMessageFactory.holdRigidBodyCurrentPose(initialFullRobotModel.getHand(robotSide.getOppositeSide()),
                                                                                                                                          keyFrameTimes);
      KinematicsPlanningToolboxInputMessage inputMessage = new KinematicsPlanningToolboxInputMessage();
      inputMessage.getRigidBodyMessages().add().set(endEffectorMessage);
      inputMessage.getRigidBodyMessages().add().set(holdAnotherHandMessage);
      System.out.println("submit");
      commandInputManager.submitMessage(inputMessage);
      System.out.println("submitted");

      if (inputMessage.getKinematicsConfigurationMessage() == null)
         System.out.println("null");
      if (inputMessage.getCenterOfMassMessage() == null)
         System.out.println("null");

      RobotConfigurationData robotConfigurationData = AvatarHumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(AvatarHumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(true, true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations);
   }

   @Test
   public void testLinearInterpolatedTrajectory() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      RobotSide robotSide = RobotSide.LEFT;
      RigidBodyBasics endEffector = initialFullRobotModel.getHand(robotSide);
      double trajectoryTime = 5.0;
      int numberOfKeyFrames = 10;
      FramePose3D initialPose = new FramePose3D(endEffector.getBodyFixedFrame());
      FramePose3D desiredPose = new FramePose3D(endEffector.getBodyFixedFrame(), new Point3D(0.1, 0.1, 0.6), new AxisAngle(1.0, 0.0, 0.0, 0.5 * Math.PI));
      initialPose.changeFrame(worldFrame);
      desiredPose.changeFrame(worldFrame);

      TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
      List<Pose3DReadOnly> keyFramePoses = new ArrayList<Pose3DReadOnly>();
      List<Point3DReadOnly> desiredCOMPoints = new ArrayList<Point3DReadOnly>();

      for (int i = 0; i < numberOfKeyFrames; i++)
      {
         double alpha = (i + 1) / (double) (numberOfKeyFrames);
         keyFrameTimes.add(alpha * trajectoryTime);
         Pose3D pose = new Pose3D(initialPose);
         pose.interpolate(desiredPose, alpha);
         keyFramePoses.add(pose);
         desiredCOMPoints.add(new Point3D());
         if (visualize)
            scs.addStaticLinkGraphics(createEndEffectorKeyFrameVisualization(pose));
      }

      KinematicsPlanningToolboxRigidBodyMessage endEffectorMessage = HumanoidMessageTools.createKinematicsPlanningToolboxRigidBodyMessage(endEffector,
                                                                                                                                          keyFrameTimes,
                                                                                                                                          keyFramePoses);
      KinematicsPlanningToolboxCenterOfMassMessage comMessage = HumanoidMessageTools.createKinematicsPlanningToolboxCenterOfMassMessage(keyFrameTimes,
                                                                                                                                        desiredCOMPoints);

      endEffectorMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      endEffectorMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));

      SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
      selectionMatrix.selectZAxis(false);
      comMessage.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix));
      comMessage.getWeights().set(MessageTools.createWeightMatrix3DMessage(1.0));

      commandInputManager.submitMessage(endEffectorMessage);
      commandInputManager.submitMessage(comMessage);

      KinematicsPlanningToolboxRigidBodyMessage holdAnotherHandMessage = KinematicsPlanningToolboxMessageFactory.holdRigidBodyCurrentPose(initialFullRobotModel.getHand(robotSide.getOppositeSide()),
                                                                                                                                          keyFrameTimes);
      commandInputManager.submitMessage(holdAnotherHandMessage);

      RobotConfigurationData robotConfigurationData = AvatarHumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(AvatarHumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(true, true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations);
   }

   @Test
   public void testReachToAPoint() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      RobotSide robotSide = RobotSide.LEFT;
      RigidBodyBasics endEffector = initialFullRobotModel.getHand(robotSide);
      double trajectoryTime = 5.0;

      Pose3D desiredPose = new Pose3D();
      desiredPose.setPosition(0.5, 0.3, 1.0);
      desiredPose.appendYawRotation(-0.5 * Math.PI);
      desiredPose.appendPitchRotation(0.5 * Math.PI);
      desiredPose.appendYawRotation(0.2 * Math.PI);
      TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
      List<Pose3DReadOnly> keyFramePoses = new ArrayList<Pose3DReadOnly>();

      keyFrameTimes.add(trajectoryTime);
      Pose3D pose = new Pose3D(desiredPose);
      keyFramePoses.add(pose);
      if (visualize)
         scs.addStaticLinkGraphics(createEndEffectorKeyFrameVisualization(pose));

      KinematicsPlanningToolboxRigidBodyMessage endEffectorMessage = HumanoidMessageTools.createKinematicsPlanningToolboxRigidBodyMessage(endEffector,
                                                                                                                                          keyFrameTimes,
                                                                                                                                          keyFramePoses);

      endEffectorMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      endEffectorMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));

      commandInputManager.submitMessage(endEffectorMessage);

      RobotConfigurationData robotConfigurationData = AvatarHumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(AvatarHumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(true, true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations);
   }

   @Test
   public void testDifferentDistanceBetweenKeyFrames() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      RobotSide robotSide = RobotSide.LEFT;
      RigidBodyBasics endEffector = initialFullRobotModel.getHand(robotSide);
      double trajectoryTime = 5.0;

      FramePose3D initialPose = new FramePose3D(endEffector.getBodyFixedFrame());
      initialPose.changeFrame(worldFrame);

      FramePose3D wayPointOne = new FramePose3D(initialPose);
      wayPointOne.appendTranslation(0.0, 0.1, 0.0);
      FramePose3D wayPointTwo = new FramePose3D(initialPose);
      wayPointTwo.appendTranslation(0.0, 0.4, 0.0);
      FramePose3D wayPointThree = new FramePose3D(initialPose);
      wayPointThree.appendTranslation(0.0, 0.4, 0.1);

      TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
      List<Pose3DReadOnly> keyFramePoses = new ArrayList<Pose3DReadOnly>();

      keyFrameTimes.add(trajectoryTime * 0.2);
      keyFrameTimes.add(trajectoryTime * 0.5);
      keyFrameTimes.add(trajectoryTime * 0.3);

      keyFramePoses.add(wayPointOne);
      keyFramePoses.add(wayPointTwo);
      keyFramePoses.add(wayPointThree);

      for (int i = 0; i < keyFramePoses.size(); i++)
         if (visualize)
            scs.addStaticLinkGraphics(createEndEffectorKeyFrameVisualization(keyFramePoses.get(i)));

      KinematicsPlanningToolboxRigidBodyMessage endEffectorMessage = HumanoidMessageTools.createKinematicsPlanningToolboxRigidBodyMessage(endEffector,
                                                                                                                                          keyFrameTimes,
                                                                                                                                          keyFramePoses);

      endEffectorMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      endEffectorMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));

      commandInputManager.submitMessage(endEffectorMessage);

      RobotConfigurationData robotConfigurationData = AvatarHumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(AvatarHumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(true, true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations);
   }

   private static Graphics3DObject createEndEffectorKeyFrameVisualization(Pose3DReadOnly pose)
   {
      Graphics3DObject object = new Graphics3DObject();
      object.transform(new RigidBodyTransform(pose.getOrientation(), pose.getPosition()));
      object.addSphere(0.01);

      return object;
   }

   private void trackSolution()
   {
      assertFalse("Poor solution quality: " + toolboxController.getSolution().getSolutionQuality(), toolboxController.getSolution().getSolutionQuality() < 0.0);
   }

   private void runKinematicsPlanningToolboxController(int numberOfIterations) throws SimulationExceededMaximumTimeException, UnreasonableAccelerationException
   {
      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         for (int i = 0; !toolboxController.isDone() && i < numberOfIterations; i++)
            if (visualize)
               scs.simulateOneTimeStep();
      }
      else
      {
         for (int i = 0; !toolboxController.isDone() && i < numberOfIterations; i++)
         {
            toolboxUpdater.doControl();
         }
      }
      trackSolution();
   }

   private FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, robotModel.getJointMap());
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, null, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null, null, null);
      drcPerfectSensorReaderFactory.getSensorReader().read();
      return initialFullRobotModel;
   }

   private RobotController createToolboxUpdater()
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot, toolboxController.getDesiredFullRobotModel());

         @Override
         public void doControl()
         {
            if (!initializationSucceeded.getBooleanValue())
               initializationSucceeded.set(toolboxController.initialize());

            if (initializationSucceeded.getBooleanValue())
            {
               try
               {
                  toolboxController.updateInternal();
               }
               catch (Exception e)
               {
                  e.printStackTrace();
               }
               jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
               numberOfIterations.increment();
            }
         }

         @Override
         public void initialize()
         {
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return mainRegistry;
         }

         @Override
         public String getName()
         {
            return mainRegistry.getName();
         }

         @Override
         public String getDescription()
         {
            return null;
         }
      };
   }
}
