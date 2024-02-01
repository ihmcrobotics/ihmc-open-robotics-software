package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import toolbox_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage;
import toolbox_msgs.msg.dds.KinematicsPlanningToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

@Tag("humanoid-toolbox")
public abstract class AvatarKinematicsPlanningToolboxControllerTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final MaterialDefinition ghostMaterial = new MaterialDefinition(ColorDefinitions.Yellow().derive(0, 1, 1, 0.25));
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();
   static
   {
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private CommandInputManager commandInputManager;
   private YoRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private KinematicsPlanningToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;

   private SimulationConstructionSet2 scs;

   private Robot robot;
   private Robot ghost;
   private Controller toolboxUpdater;

   /**
    * Returns a separate instance of the robot model that will be modified in this test to create a
    * ghost robot.
    */
   public abstract DRCRobotModel getGhostRobotModel();

   @BeforeEach
   public void setup()
   {
      mainRegistry = new YoRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();

      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
      commandInputManager = new CommandInputManager(KinematicsPlanningToolboxModule.supportedCommands());

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsPlanningToolboxModule.supportedStatus());

      toolboxController = new KinematicsPlanningToolboxController(robotModel,
                                                                  desiredFullRobotModel,
                                                                  commandInputManager,
                                                                  statusOutputManager,
                                                                  yoGraphicsListRegistry,
                                                                  mainRegistry);
      commandInputManager.registerConversionHelper(new KinematicsPlanningToolboxCommandConverter(desiredFullRobotModel, toolboxController.getDesiredReferenceFrames()));

      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      robotDefinition.ignoreAllJoints();
      robot = new Robot(robotDefinition, SimulationConstructionSet2.inertialFrame);
      toolboxUpdater = createToolboxUpdater();
      robot.addController(toolboxUpdater);

      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      RobotDefinition ghostRobotDefinition = ghostRobotModel.getRobotDefinition();
      ghostRobotDefinition.ignoreAllJoints();
      ghostRobotDefinition.setName("Ghost");
      RobotDefinitionTools.setRobotDefinitionMaterial(ghostRobotDefinition, ghostMaterial);
      ghost = new Robot(ghostRobotDefinition, SimulationConstructionSet2.inertialFrame);
      hideGhost();

      if (visualize)
      {
         scs = new SimulationConstructionSet2();
         scs.addRobot(robot);
         scs.addRobot(ghost);
         scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));
         scs.start(true, true, true);
         scs.setCameraFocusPosition(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
      }
   }

   private void hideGhost()
   {
      ghost.getFloatingRootJoint().getJointPose().getPosition().setX(Double.POSITIVE_INFINITY);
      ghost.updateFrames();
   }

   private void snapGhostToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      MultiBodySystemTools.copyJointsState(fullHumanoidRobotModel.getRootJoint().subtreeList(), ghost.getAllJoints(), JointStateType.CONFIGURATION);
      ghost.updateFrames();
   }

   @AfterEach
   public void tearDown()
   {
      if (visualize)
         scs.waitUntilVisualizerDown();

      if (mainRegistry != null)
      {
         mainRegistry.clear();
         mainRegistry = null;
      }

      initializationSucceeded = null;

      yoGraphicsListRegistry = null;

      commandInputManager = null;

      toolboxController = null;

      robot = null;
      toolboxUpdater = null;

      if (scs != null)
      {
         scs.shutdownSession();
         scs = null;
      }

      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testDualHandTrajectory()
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
            scs.addStaticVisual(createEndEffectorKeyFrameVisualization(pose));
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

      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(initialFullRobotModel,
                                                                                                                              getRobotModel(),
                                                                                                                              true,
                                                                                                                              true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations, KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_OPTIMAL_SOLUTION);
   }

   @Test
   public void testLinearInterpolatedTrajectory()
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
            scs.addStaticVisual(createEndEffectorKeyFrameVisualization(pose));
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

      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(initialFullRobotModel,
                                                                                                                              getRobotModel(),
                                                                                                                              true,
                                                                                                                              true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations, KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_OPTIMAL_SOLUTION);
   }

   @Test
   public void testReachToAPoint()
   {
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      RobotSide robotSide = RobotSide.LEFT;
      RigidBodyBasics endEffector = initialFullRobotModel.getHand(robotSide);
      double trajectoryTime = 5.0;

      Pose3D desiredPose = new Pose3D();
      desiredPose.getPosition().set(0.5, 0.3, 1.0);
      desiredPose.appendYawRotation(-0.5 * Math.PI);
      desiredPose.appendPitchRotation(0.5 * Math.PI);
      desiredPose.appendYawRotation(0.2 * Math.PI);
      TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
      List<Pose3DReadOnly> keyFramePoses = new ArrayList<Pose3DReadOnly>();

      keyFrameTimes.add(trajectoryTime);
      Pose3D pose = new Pose3D(desiredPose);
      keyFramePoses.add(pose);
      if (visualize)
         scs.addStaticVisual(createEndEffectorKeyFrameVisualization(pose));

      KinematicsPlanningToolboxRigidBodyMessage endEffectorMessage = HumanoidMessageTools.createKinematicsPlanningToolboxRigidBodyMessage(endEffector,
                                                                                                                                          keyFrameTimes,
                                                                                                                                          keyFramePoses);

      endEffectorMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      endEffectorMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));

      commandInputManager.submitMessage(endEffectorMessage);

      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(initialFullRobotModel,
                                                                                                                              getRobotModel(),
                                                                                                                              true,
                                                                                                                              true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations, KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_OPTIMAL_SOLUTION);
   }

   @Test
   public void testDifferentDistanceBetweenKeyFrames()
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
      keyFrameTimes.add(trajectoryTime * 1.0);

      keyFramePoses.add(wayPointOne);
      keyFramePoses.add(wayPointTwo);
      keyFramePoses.add(wayPointThree);

      for (int i = 0; i < keyFramePoses.size(); i++)
         if (visualize)
            scs.addStaticVisual(createEndEffectorKeyFrameVisualization(keyFramePoses.get(i)));

      KinematicsPlanningToolboxRigidBodyMessage endEffectorMessage = HumanoidMessageTools.createKinematicsPlanningToolboxRigidBodyMessage(endEffector,
                                                                                                                                          keyFrameTimes,
                                                                                                                                          keyFramePoses);

      endEffectorMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      endEffectorMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));

      commandInputManager.submitMessage(endEffectorMessage);

      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(initialFullRobotModel,
                                                                                                                              getRobotModel(),
                                                                                                                              true,
                                                                                                                              true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations, KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_OPTIMAL_SOLUTION);
   }

   @Test
   public void testLastKeyFrameBadPositionPlanning()
   {
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBodyBasics endEffector = initialFullRobotModel.getHand(robotSide);
      ReferenceFrame bodyFixedFrame = endEffector.getBodyFixedFrame();

      FramePose3D initialPose = new FramePose3D(bodyFixedFrame);
      initialPose.changeFrame(worldFrame);

      TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
      List<Pose3DReadOnly> desiredPoses = new ArrayList<>();

      Pose3D desiredPoseInHandFrame = new Pose3D();
      desiredPoseInHandFrame.appendTranslation(0.0, 0.0, 0.05);
      FramePose3D desiredPoseOne = new FramePose3D(bodyFixedFrame, desiredPoseInHandFrame);
      desiredPoseInHandFrame.appendTranslation(0.0, 0.0, 0.05);
      FramePose3D desiredPoseTwo = new FramePose3D(bodyFixedFrame, desiredPoseInHandFrame);
      desiredPoseInHandFrame.appendTranslation(0.0, 0.0, 0.05);
      FramePose3D desiredPoseThree = new FramePose3D(bodyFixedFrame, desiredPoseInHandFrame);
      desiredPoseInHandFrame.appendTranslation(0.0, 0.0, 1.0);
      FramePose3D desiredPoseBad = new FramePose3D(bodyFixedFrame, desiredPoseInHandFrame);

      desiredPoseOne.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPoseTwo.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPoseThree.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPoseBad.changeFrame(ReferenceFrame.getWorldFrame());

      keyFrameTimes.add(1.0);
      keyFrameTimes.add(2.0);
      keyFrameTimes.add(3.0);
      keyFrameTimes.add(4.0);
      desiredPoses.add(desiredPoseOne);
      desiredPoses.add(desiredPoseTwo);
      desiredPoses.add(desiredPoseThree);
      desiredPoses.add(desiredPoseBad);

      for (int i = 0; i < desiredPoses.size(); i++)
         if (visualize)
            scs.addStaticVisual(createEndEffectorKeyFrameVisualization(desiredPoses.get(i)));

      KinematicsPlanningToolboxRigidBodyMessage endEffectorMessage = HumanoidMessageTools.createKinematicsPlanningToolboxRigidBodyMessage(endEffector,
                                                                                                                                          keyFrameTimes,
                                                                                                                                          desiredPoses);

      endEffectorMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      endEffectorMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));

      commandInputManager.submitMessage(endEffectorMessage);

      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus(initialFullRobotModel,
                                                                                                                              getRobotModel(),
                                                                                                                              true,
                                                                                                                              true));

      int numberOfIterations = 350;

      runKinematicsPlanningToolboxController(numberOfIterations, KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_UNREACHABLE_KEYFRAME);
   }

   private static VisualDefinition createEndEffectorKeyFrameVisualization(Pose3DReadOnly pose)
   {
      return new VisualDefinition(new RigidBodyTransform(pose.getOrientation(), pose.getPosition()),
                                  new Sphere3DDefinition(0.01),
                                  new MaterialDefinition(ColorDefinitions.Black()));
   }

   private void trackSolution(int expectedResult)
   {
      assertTrue(expectedResult == toolboxController.getSolution().getPlanId());
   }

   private void runKinematicsPlanningToolboxController(int numberOfIterations, int expectedResult)
   {
      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         for (int i = 0; !toolboxController.isDone() && i < numberOfIterations; i++)
            scs.simulateNow(1);
      }
      else
      {
         for (int i = 0; !toolboxController.isDone() && i < numberOfIterations; i++)
         {
            toolboxUpdater.doControl();
         }
      }
      trackSolution(expectedResult);
   }

   private FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeFullRobotModel(initialFullRobotModel);
      initialFullRobotModel.updateFrames();
      return initialFullRobotModel;
   }

   private Controller createToolboxUpdater()
   {
      return new Controller()
      {
         private final List<? extends JointBasics> allToolboxJoints = toolboxController.getDesiredFullRobotModel().getRootJoint().subtreeList();

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
               MultiBodySystemTools.copyJointsState(allToolboxJoints, robot.getAllJoints(), JointStateType.CONFIGURATION);
               robot.updateFrames();

               numberOfIterations.increment();
            }
         }

         @Override
         public void initialize()
         {
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return mainRegistry;
         }

         @Override
         public String getName()
         {
            return mainRegistry.getName();
         }
      };
   }
}
