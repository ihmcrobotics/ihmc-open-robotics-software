package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarHumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarHumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.PITCH;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.ROLL;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.YAW;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage;
import static us.ihmc.robotics.Assert.assertNotNull;
import static us.ihmc.robotics.Assert.fail;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.IntStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.ReachingManifoldMessage;
import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class AvatarWholeBodyTrajectoryToolboxControllerTest implements MultiRobotTestInterface
{
   protected static final boolean VERBOSE = false;

   private static final AppearanceDefinition ghostApperance = YoAppearance.DarkGreen();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();
   static
   {
      simulationTestingParameters.setKeepSCSUp(false);
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private CommandInputManager commandInputManager;
   private StatusMessageOutputManager statusOutputManager;
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private WholeBodyTrajectoryToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;

   private SimulationConstructionSet scs;

   private HumanoidFloatingRootJointRobot robot;
   private HumanoidFloatingRootJointRobot ghost;
   private RobotController toolboxUpdater;

   private WholeBodyTrajectoryToolboxCommandConverter commandConversionHelper;
   private KinematicsToolboxOutputConverter converter;

   /**
    * Returns a separate instance of the robot model that will be modified in this test to create a
    * ghost robot.
    */
   public abstract DRCRobotModel getGhostRobotModel();

   private static final double TRACKING_TRAJECTORY_POSITION_ERROR_THRESHOLD = 0.05;
   private static final double TRACKING_TRAJECTORY_ORIENTATION_ERROR_THRESHOLD = 0.05;

   @BeforeEach
   public void setup()
   {
      mainRegistry = new YoVariableRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();

      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
      commandInputManager = new CommandInputManager(WholeBodyTrajectoryToolboxModule.supportedCommands());
      commandConversionHelper = new WholeBodyTrajectoryToolboxCommandConverter(desiredFullRobotModel);
      commandInputManager.registerConversionHelper(commandConversionHelper);
      commandInputManager.registerMessageUnpacker(WholeBodyTrajectoryToolboxMessage.class, MessageUnpackingTools.createWholeBodyTrajectoryToolboxMessageUnpacker());

      converter = new KinematicsToolboxOutputConverter(robotModel);

      statusOutputManager = new StatusMessageOutputManager(WholeBodyTrajectoryToolboxModule.supportedStatus());

      toolboxController = new WholeBodyTrajectoryToolboxController(getRobotModel(), desiredFullRobotModel, commandInputManager, statusOutputManager,
                                                                   mainRegistry, yoGraphicsListRegistry, visualize);

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
      }
   }

   private void hideGhost()
   {
      ghost.setPositionInWorld(new Point3D(-100.0, -100.0, -100.0));
      ghost.update();
   }

   private void hideRobot()
   {
      robot.setPositionInWorld(new Point3D(-100.0, -100.0, -100.0));
      robot.update();
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
      statusOutputManager = null;

      toolboxController = null;

      robot = null;
      toolboxUpdater = null;

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }
   }

   @Test
   public void testOneBigCircle() throws Exception, UnreasonableAccelerationException
   {
      // Trajectory parameters
      double trajectoryTime = 10.0;
      double circleRadius = 0.6; // Valkyrie, enable ypr, is available for 0.5 radius.
      Point3D circleCenter = new Point3D(0.55, 0.2, 1.0);
      Quaternion circleOrientation = new Quaternion();
      circleOrientation.appendYawRotation(Math.PI * 0.0);
      Quaternion handOrientation = new Quaternion(circleOrientation);

      // WBT toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message, exploration message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == RobotSide.LEFT)
         {
            RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

            boolean ccw;
            if (robotSide == RobotSide.RIGHT)
               ccw = false;
            else
               ccw = true;
            FunctionTrajectory handFunction = time -> computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenter, circleOrientation,
                                                                              handOrientation, ccw, 0.0);

            SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
            selectionMatrix.resetSelection();
            selectionMatrix.clearAngularSelection();
            WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction, selectionMatrix);
            Pose3D controlFramePose = new Pose3D();

            trajectory.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
            trajectory.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());

            handTrajectories.add(trajectory);
            ConfigurationSpaceName[] handConfigurations = {};
            RigidBodyExplorationConfigurationMessage rigidBodyConfiguration = HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand,
                                                                                                                                                  handConfigurations);

            rigidBodyConfigurations.add(rigidBodyConfiguration);

            if (visualize)
               scs.addStaticLinkGraphics(createFunctionTrajectoryVisualization(handFunction, 0.0, trajectoryTime, timeResolution, 0.01,
                                                                               YoAppearance.AliceBlue()));
         }
      }

      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null,
                                                                                                               rigidBodyConfigurations);

      // run toolbox
      runTrajectoryTest(message, 100000);
   }

   @Test
   public void testHandCirclePositionAndYaw() throws Exception, UnreasonableAccelerationException
   {
      // Trajectory parameters
      double trajectoryTime = 10.0;
      double circleRadius = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(new Point3D(0.55, 0.4, 0.9), new Point3D(0.55, -0.4, 0.9));
      Quaternion circleOrientation = new Quaternion();
      circleOrientation.appendYawRotation(Math.PI * 0.0);
      Quaternion handOrientation = new Quaternion(circleOrientation);

      // WBT toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message, exploration message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         boolean ccw;
         if (robotSide == RobotSide.RIGHT)
            ccw = false;
         else
            ccw = true;
         FunctionTrajectory handFunction = time -> computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenters.get(robotSide), circleOrientation,
                                                                           handOrientation, ccw, 0.0);

         SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
         selectionMatrix.resetSelection();
         WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction, selectionMatrix);
         Pose3D controlFramePose = new Pose3D();

         trajectory.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
         trajectory.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());

         handTrajectories.add(trajectory);
         ConfigurationSpaceName[] handConfigurations = {ConfigurationSpaceName.YAW};
         RigidBodyExplorationConfigurationMessage rigidBodyConfiguration = HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand,
                                                                                                                                               handConfigurations);

         rigidBodyConfigurations.add(rigidBodyConfiguration);

         if (visualize)
            scs.addStaticLinkGraphics(createFunctionTrajectoryVisualization(handFunction, 0.0, trajectoryTime, timeResolution, 0.01, YoAppearance.AliceBlue()));
      }

      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null,
                                                                                                               rigidBodyConfigurations);

      // run toolbox
      runTrajectoryTest(message, 100000);
   }

   @Test
   public void testHandCirclePositionAndYawPitchRoll() throws Exception, UnreasonableAccelerationException
   {
      // Trajectory parameters
      double trajectoryTime = 5.0;
      double circleRadius = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(new Point3D(0.6, 0.35, 1.0), new Point3D(0.6, -0.35, 1.0));
      Quaternion circleOrientation = new Quaternion();
      circleOrientation.appendYawRotation(Math.PI * 0.05);
      Quaternion handOrientation = new Quaternion(circleOrientation);

      // WBT toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message, exploration message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         // orientation is defined
         boolean ccw;
         if (robotSide == RobotSide.RIGHT)
            ccw = false;
         else
            ccw = true;
         FunctionTrajectory handFunction = time -> computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenters.get(robotSide), circleOrientation,
                                                                           handOrientation, ccw, 0.0);

         SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
         selectionMatrix.resetSelection();
         WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction, selectionMatrix);
         Pose3D controlFramePose = new Pose3D();

         trajectory.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
         trajectory.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());

         handTrajectories.add(trajectory);

         ConfigurationSpaceName[] spaces = {YAW, PITCH, ROLL};

         rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));
      }

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null,
                                                                                                               rigidBodyConfigurations);

      // run toolbox
      runTrajectoryTest(message, maxNumberOfIterations);
   }

   protected void runTrajectoryTest(WholeBodyTrajectoryToolboxMessage message, int maxNumberOfIterations) throws UnreasonableAccelerationException
   {
      List<WaypointBasedTrajectoryMessage> endEffectorTrajectories = message.getEndEffectorTrajectories();
      double t0 = Double.POSITIVE_INFINITY;
      double tf = Double.NEGATIVE_INFINITY;

      if (endEffectorTrajectories != null)
      {
         for (int i = 0; i < endEffectorTrajectories.size(); i++)
         {
            WaypointBasedTrajectoryMessage trajectoryMessage = endEffectorTrajectories.get(i);
            t0 = Math.min(t0, trajectoryMessage.getWaypointTimes().get(0));
            tf = Math.max(t0, trajectoryMessage.getWaypointTimes().get(trajectoryMessage.getWaypoints().size() - 1));

            SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
            // Visualize the position part if it is commanded
            selectionMatrix.resetSelection();
            SelectionMatrix3DMessage angularSelection = trajectoryMessage.getAngularSelectionMatrix();
            SelectionMatrix3DMessage linearSelection = trajectoryMessage.getLinearSelectionMatrix();
            selectionMatrix.setAngularAxisSelection(angularSelection.getXSelected(), angularSelection.getYSelected(), angularSelection.getZSelected());
            selectionMatrix.setLinearAxisSelection(linearSelection.getXSelected(), linearSelection.getYSelected(), linearSelection.getZSelected());

            if (!selectionMatrix.isLinearXSelected() && !selectionMatrix.isLinearYSelected() && !selectionMatrix.isLinearZSelected())
               continue; // The position part is not dictated by trajectory, let's not visualize.

            if (visualize)
               scs.addStaticLinkGraphics(createTrajectoryMessageVisualization(trajectoryMessage, 0.01, YoAppearance.AliceBlue()));
         }
      }

      double trajectoryTime = tf - t0;

      commandInputManager.submitMessage(message);

      WholeBodyTrajectoryToolboxOutputStatus solution = runToolboxController(maxNumberOfIterations);

      if (numberOfIterations.getIntegerValue() < maxNumberOfIterations - 1)
         assertNotNull("The toolbox is done but did not report a solution.", solution);
      else
         fail("The toolbox has run for " + maxNumberOfIterations + " without converging nor aborting.");

      if (solution.getPlanningResult() == 4)
      {
         if (visualize)
            visualizeSolution(solution, trajectoryTime / 1000.0);

         trackingTrajectoryWithOutput(message, solution);
      }
      else
      {
         fail("planning result " + solution.getPlanningResult());
      }
   }

   protected void runReachingTest(WholeBodyTrajectoryToolboxMessage message, int maxNumberOfIterations) throws UnreasonableAccelerationException
   {
      List<ReachingManifoldMessage> reachingManifolds = message.getReachingManifolds();
      if (reachingManifolds != null)
      {
         for (int i = 0; i < reachingManifolds.size(); i++)
         {
            ReachingManifoldMessage manifold = reachingManifolds.get(i);
            if (visualize)
               scs.addStaticLinkGraphics(createTrajectoryMessageVisualization(manifold, 0.01, YoAppearance.AliceBlue()));
         }
      }

      commandInputManager.submitMessage(message);

      WholeBodyTrajectoryToolboxOutputStatus solution = runToolboxController(maxNumberOfIterations);

      // TODO
      // tracking

      if (numberOfIterations.getIntegerValue() < maxNumberOfIterations - 1)
         assertNotNull("The toolbox is done but did not report a solution.", solution);
      else
         fail("The toolbox has run for " + maxNumberOfIterations + " without converging nor aborting.");

      if (solution.getPlanningResult() == 4)
      {
         if (visualize)
            visualizeSolution(solution, 10.0 / 1000.0);

         trackingTrajectoryWithOutput(message, solution);
      }
      else
      {
         fail("planning result " + solution.getPlanningResult());
      }
   }

   public void trackingTrajectoryWithOutput(WholeBodyTrajectoryToolboxMessage message, WholeBodyTrajectoryToolboxOutputStatus solution)
   {
      List<WaypointBasedTrajectoryMessage> wayPointBasedTrajectoryMessages = message.getEndEffectorTrajectories();

      // for every configurations in solution.
      int numberOfConfigurations = solution.getRobotConfigurations().size();
      for (int j = 0; j < numberOfConfigurations; j++)
      {
         // get full robot model.
         KinematicsToolboxOutputStatus configuration = solution.getRobotConfigurations().get(j);
         converter.updateFullRobotModel(configuration);
         FullHumanoidRobotModel outputFullRobotModel = converter.getFullRobotModel();

         double configurationTime = solution.getTrajectoryTimes().get(j);

         // for all way point based trajectory messages.
         for (int i = 0; i < wayPointBasedTrajectoryMessages.size(); i++)
         {
            WaypointBasedTrajectoryMessage trajectory = wayPointBasedTrajectoryMessages.get(i);
            RigidBodyExplorationConfigurationMessage explorationMessage = getRigidBodyExplorationConfigurationMessageHasSameHashCode(message.getExplorationConfigurations(),
                                                                                                                                     trajectory);
            RigidBodyBasics rigidBodyOftrajectory = commandConversionHelper.getRigidBody(trajectory.getEndEffectorHashCode());

            RigidBodyBasics rigidBodyOfOutputFullRobotModel = getRigidBodyHasSameName(outputFullRobotModel, rigidBodyOftrajectory);

            if (rigidBodyOfOutputFullRobotModel == null)
            {
               if (VERBOSE)
                  PrintTools.info("there is no rigid body");
               fail("there is no rigid body");
            }
            else
            {
               RigidBodyTransform solutionRigidBodyTransform = rigidBodyOfOutputFullRobotModel.getBodyFixedFrame().getTransformToWorldFrame();
               Pose3D solutionRigidBodyPose = new Pose3D(solutionRigidBodyTransform);

               if (trajectory.getControlFramePositionInEndEffector() != null)
                  solutionRigidBodyPose.appendTransform(new RigidBodyTransform(new Quaternion(), trajectory.getControlFramePositionInEndEffector()));
               if (trajectory.getControlFrameOrientationInEndEffector() != null)
                  solutionRigidBodyPose.appendTransform(new RigidBodyTransform(trajectory.getControlFrameOrientationInEndEffector(), new Point3D()));

               Pose3D givenRigidBodyPose = HumanoidMessageTools.unpackPose(trajectory, configurationTime);

               double positionError = WholeBodyTrajectoryToolboxHelper.computeTrajectoryPositionError(solutionRigidBodyPose, givenRigidBodyPose,
                                                                                                      explorationMessage, trajectory);

               double orientationError = WholeBodyTrajectoryToolboxHelper.computeTrajectoryOrientationError(solutionRigidBodyPose, givenRigidBodyPose,
                                                                                                            explorationMessage, trajectory);

               if (VERBOSE)
                  PrintTools.info("" + positionError + " " + orientationError);

               if (positionError > TRACKING_TRAJECTORY_POSITION_ERROR_THRESHOLD || orientationError > TRACKING_TRAJECTORY_ORIENTATION_ERROR_THRESHOLD)
               {
                  PrintTools.info("rigid body of the solution is far from the given trajectory");
                  fail("rigid body of the solution is far from the given trajectory");
               }

            }
         }
      }
   }

   private RigidBodyBasics getRigidBodyHasSameName(FullHumanoidRobotModel fullRobotModel, RigidBodyBasics givenRigidBody)
   {
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(fullRobotModel.getElevator());
      RigidBodyBasics[] allRigidBodies = rootBody.subtreeArray();
      for (RigidBodyBasics rigidBody : allRigidBodies)
         if (givenRigidBody.getName().equals(rigidBody.getName()))
            return rigidBody;
      return null;
   }

   private RigidBodyExplorationConfigurationMessage getRigidBodyExplorationConfigurationMessageHasSameHashCode(List<RigidBodyExplorationConfigurationMessage> rigidBodyExplorationConfigurationMessages,
                                                                                                               WaypointBasedTrajectoryMessage trajectory)
   {
      for (int i = 0; i < rigidBodyExplorationConfigurationMessages.size(); i++)
      {
         RigidBodyExplorationConfigurationMessage message = rigidBodyExplorationConfigurationMessages.get(i);
         if (trajectory.getEndEffectorHashCode() == message.getRigidBodyHashCode())
            return message;
      }
      return null;
   }

   // TODO
   // Is this for testing ahead put message on planner?
   private SideDependentList<Pose3D> computePrivilegedHandPosesAtPositions(SideDependentList<Point3D> desiredPositions)
   {
      CommandInputManager commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());
      FullHumanoidRobotModel desiredFullRobotModel = getRobotModel().createFullRobotModel();

      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel));
      HumanoidKinematicsToolboxController whik = new HumanoidKinematicsToolboxController(commandInputManager, statusOutputManager, desiredFullRobotModel,
                                                                                         new YoGraphicsListRegistry(), new YoVariableRegistry("dummy"));

      FullHumanoidRobotModel fullRobotModelAtInitialConfiguration = createFullRobotModelWithArmsAtMidRange();
      whik.updateRobotConfigurationData(extractRobotConfigurationData(fullRobotModelAtInitialConfiguration));
      whik.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(true, true));

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = desiredFullRobotModel.getHand(robotSide);
         KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, desiredPositions.get(robotSide));
         message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         commandInputManager.submitMessage(message);
      }

      commandInputManager.submitMessage(KinematicsToolboxMessageFactory.holdRigidBodyCurrentOrientation(desiredFullRobotModel.getChest()));

      int counter = 0;
      int maxIterations = 500;

      while (counter++ <= maxIterations)
      {
         whik.update();
         System.out.println(whik.getSolution().getSolutionQuality());
         snapGhostToFullRobotModel(desiredFullRobotModel);
         scs.tickAndUpdate();
      }

      if (whik.getSolution().getSolutionQuality() > 0.005)
      {
         return null;
      }

      SideDependentList<Pose3D> handPoses = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         Pose3D handPose = new Pose3D();
         handPoses.put(robotSide, handPose);
         RigidBodyBasics hand = desiredFullRobotModel.getHand(robotSide);
         RigidBodyTransform transformToWorldFrame = hand.getBodyFixedFrame().getTransformToWorldFrame();
         handPose.set(transformToWorldFrame);
      }
      return handPoses;
   }

   private static Pose3D computeCircleTrajectory(double time, double trajectoryTime, double circleRadius, Point3DReadOnly circleCenter,
                                                 Quaternion circleRotation, QuaternionReadOnly constantOrientation, boolean ccw, double phase)
   {
      double theta = (ccw ? -time : time) / trajectoryTime * 2.0 * Math.PI + phase;
      double z = circleRadius * Math.sin(theta);
      double y = circleRadius * Math.cos(theta);
      Point3D point = new Point3D(0.0, y, z);
      circleRotation.transform(point);
      point.add(circleCenter);

      return new Pose3D(point, constantOrientation);
   }

   private static Graphics3DObject createFunctionTrajectoryVisualization(FunctionTrajectory trajectoryToVisualize, double t0, double tf, double timeResolution,
                                                                         double radius, AppearanceDefinition appearance)
   {
      int numberOfWaypoints = (int) Math.round((tf - t0) / timeResolution) + 1;
      double dT = (tf - t0) / (numberOfWaypoints - 1);

      int radialResolution = 16;
      SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(numberOfWaypoints, radialResolution, radius);
      Point3DReadOnly[] waypoints = IntStream.range(0, numberOfWaypoints).mapToDouble(i -> t0 + i * dT).mapToObj(trajectoryToVisualize::compute)
                                             .map(pose -> new Point3D(pose.getPosition())).toArray(size -> new Point3D[size]);
      segmentedLine3DMeshGenerator.compute(waypoints);

      Graphics3DObject graphics = new Graphics3DObject();
      for (MeshDataHolder mesh : segmentedLine3DMeshGenerator.getMeshDataHolders())
      {
         graphics.addMeshData(mesh, appearance);
      }
      return graphics;
   }

   private static Graphics3DObject createTrajectoryMessageVisualization(WaypointBasedTrajectoryMessage trajectoryMessage, double radius,
                                                                        AppearanceDefinition appearance)
   {
      double t0 = trajectoryMessage.getWaypointTimes().get(0);
      double tf = trajectoryMessage.getWaypointTimes().get(trajectoryMessage.getWaypoints().size() - 1);
      double timeResolution = (tf - t0) / trajectoryMessage.getWaypoints().size();
      FunctionTrajectory trajectoryToVisualize = WholeBodyTrajectoryToolboxMessageTools.createFunctionTrajectory(trajectoryMessage);
      return createFunctionTrajectoryVisualization(trajectoryToVisualize, t0, tf, timeResolution, radius, appearance);
   }

   private static Graphics3DObject createTrajectoryMessageVisualization(ReachingManifoldMessage reachingMessage, double radius, AppearanceDefinition appearance)
   {
      int configurationValueResolution = 20;
      int numberOfPoints = (int) Math.pow(configurationValueResolution, reachingMessage.getManifoldConfigurationSpaceNames().size());
      int radialResolution = 16;

      SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(numberOfPoints, radialResolution, radius);

      Point3D[] points = new Point3D[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Pose3D originPose = new Pose3D(reachingMessage.getManifoldOriginPosition(), reachingMessage.getManifoldOriginOrientation());
         double[] configurationValues = new double[reachingMessage.getManifoldConfigurationSpaceNames().size()];
         int[] configurationIndex = new int[reachingMessage.getManifoldConfigurationSpaceNames().size()];

         int tempIndex = i;
         for (int j = reachingMessage.getManifoldConfigurationSpaceNames().size(); j > 0; j--)
         {
            configurationIndex[j - 1] = (int) (tempIndex / Math.pow(configurationValueResolution, j - 1));
            tempIndex = (int) (tempIndex % Math.pow(configurationValueResolution, j - 1));
         }

         for (int j = 0; j < reachingMessage.getManifoldConfigurationSpaceNames().size(); j++)
         {
            configurationValues[j] = (reachingMessage.getManifoldUpperLimits().get(j) - reachingMessage.getManifoldLowerLimits().get(j))
                  / (configurationValueResolution - 1) * configurationIndex[j] + reachingMessage.getManifoldLowerLimits().get(j);
            switch (ConfigurationSpaceName.fromByte(reachingMessage.getManifoldConfigurationSpaceNames().get(j)))
            {
            case X:
               originPose.appendTranslation(configurationValues[j], 0.0, 0.0);
               break;
            case Y:
               originPose.appendTranslation(0.0, configurationValues[j], 0.0);
               break;
            case Z:
               originPose.appendTranslation(0.0, 0.0, configurationValues[j]);
               break;
            case ROLL:
               originPose.appendRollRotation(configurationValues[j]);
               break;
            case PITCH:
               originPose.appendPitchRotation(configurationValues[j]);
               break;
            case YAW:
               originPose.appendYawRotation(configurationValues[j]);
               break;
            default:
               break;
            }
         }

         points[i] = new Point3D(originPose.getPosition());
      }

      segmentedLine3DMeshGenerator.compute(points);

      Graphics3DObject graphics = new Graphics3DObject();
      for (MeshDataHolder mesh : segmentedLine3DMeshGenerator.getMeshDataHolders())
      {
         graphics.addMeshData(mesh, appearance);
      }

      return graphics;
   }

   private void visualizeSolution(WholeBodyTrajectoryToolboxOutputStatus solution, double timeResolution) throws UnreasonableAccelerationException
   {
      hideRobot();
      robot.getControllers().clear();

      FullHumanoidRobotModel robotForViz = getRobotModel().createFullRobotModel();
      FloatingJointBasics rootJoint = robotForViz.getRootJoint();
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(robotForViz);

      double trajectoryTime = solution.getTrajectoryTimes().get(solution.getTrajectoryTimes().size() - 1);

      double t = 0.0;

      while (t <= trajectoryTime)
      {
         t += timeResolution;
         KinematicsToolboxOutputStatus frame = findFrameFromTime(solution, t);
         MessageTools.unpackDesiredJointState(frame, rootJoint, joints);

         robotForViz.updateFrames();
         snapGhostToFullRobotModel(robotForViz);
         scs.simulateOneTimeStep();
      }
   }

   private KinematicsToolboxOutputStatus findFrameFromTime(WholeBodyTrajectoryToolboxOutputStatus outputStatus, double time)
   {
      if (time <= 0.0)
         return outputStatus.getRobotConfigurations().get(0);

      else if (time >= outputStatus.getTrajectoryTimes().get(outputStatus.getTrajectoryTimes().size() - 1))
         return outputStatus.getRobotConfigurations().get(outputStatus.getRobotConfigurations().size() - 1);
      else
      {
         double timeGap = 0.0;

         int indexOfFrame = 0;
         int numberOfTrajectoryTimes = outputStatus.getTrajectoryTimes().size();

         for (int i = 0; i < numberOfTrajectoryTimes; i++)
         {
            timeGap = time - outputStatus.getTrajectoryTimes().get(i);
            if (timeGap < 0)
            {
               indexOfFrame = i;
               break;
            }
         }

         KinematicsToolboxOutputStatus frameOne = outputStatus.getRobotConfigurations().get(indexOfFrame - 1);
         KinematicsToolboxOutputStatus frameTwo = outputStatus.getRobotConfigurations().get(indexOfFrame);

         double timeOne = outputStatus.getTrajectoryTimes().get(indexOfFrame - 1);
         double timeTwo = outputStatus.getTrajectoryTimes().get(indexOfFrame);

         double alpha = (time - timeOne) / (timeTwo - timeOne);

         return MessageTools.interpolateMessages(frameOne, frameTwo, alpha);
      }
   }

   protected FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, robotModel.getJointMap());
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, null, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null);
      drcPerfectSensorReaderFactory.getSensorReader().read();

      return initialFullRobotModel;
   }

   private FullHumanoidRobotModel createFullRobotModelWithArmsAtMidRange()
   {
      FullHumanoidRobotModel robot = createFullRobotModelAtInitialConfiguration();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = robot.getChest();
         RigidBodyBasics hand = robot.getHand(robotSide);
         Arrays.stream(MultiBodySystemTools.createOneDoFJointPath(chest, hand)).forEach(j -> setJointPositionToMidRange(j));
      }
      return robot;
   }

   private static void setJointPositionToMidRange(OneDoFJointBasics joint)
   {
      double jointLimitUpper = joint.getJointLimitUpper();
      double jointLimitLower = joint.getJointLimitLower();
      joint.setQ(0.5 * (jointLimitUpper + jointLimitLower));
   }

   private WholeBodyTrajectoryToolboxOutputStatus runToolboxController(int maxNumberOfIterations) throws UnreasonableAccelerationException
   {
      AtomicReference<WholeBodyTrajectoryToolboxOutputStatus> status = new AtomicReference<>(null);
      statusOutputManager.attachStatusMessageListener(WholeBodyTrajectoryToolboxOutputStatus.class, status::set);

      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         for (int i = 0; !toolboxController.isDone() && i < maxNumberOfIterations; i++)
            scs.simulateOneTimeStep();
      }
      else
      {
         for (int i = 0; !toolboxController.isDone() && i < maxNumberOfIterations; i++)
            toolboxUpdater.doControl();
      }
      return status.getAndSet(null);
   }

   private RobotController createToolboxUpdater()
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot, toolboxController.getSolverFullRobotModel());

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
               catch (InterruptedException | ExecutionException e)
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
