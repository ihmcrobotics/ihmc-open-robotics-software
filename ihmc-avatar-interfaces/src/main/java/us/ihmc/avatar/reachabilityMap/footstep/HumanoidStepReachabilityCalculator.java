package us.ihmc.avatar.reachabilityMap.footstep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.multiContact.CenterOfMassMotionControlAnchorDescription;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.OneDoFMotionControlAnchorDescription;
import us.ihmc.avatar.multiContact.SixDoFMotionControlAnchorDescription;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.KinematicsPlanningToolboxOptimizationSettings;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.SolutionQualityConvergenceDetector;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.IKRobotStateUpdater;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.*;

/**
 * Pattern matched off of HumanoidKinematicsToolboxControllerTest. Check there for reference if needed.
 */
public abstract class HumanoidStepReachabilityCalculator
{

   private enum Mode
   {
      HAND_POSE, TEST_SINGLE_STEP, TEST_MULTIPLE_STEPS, TEST_VISUALIZATION, TEST_WRITE_SCRIPT, TEST_LOAD_SCRIPT
   }

   private static final Mode mode = Mode.TEST_MULTIPLE_STEPS;

   private static final double COM_WEIGHT = 1.0;
   private static final double RIGID_BODY_FEET_WEIGHT = 40.0;
   private static final double RIGID_BODY_OTHER_WEIGHT = 20.0;
   private static final double JOINT_WEIGHT = 10.0;
   private static final double SOLUTION_QUALITY_THRESHOLD = 2.2;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final MaterialDefinition ghostMaterial = new MaterialDefinition(ColorDefinitions.Yellow().derive(0, 1, 1, 0.25));
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();

   static
   {
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private final CommandInputManager commandInputManager;
   private final YoRegistry mainRegistry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final HumanoidKinematicsToolboxController toolboxController;
   private final KinematicsPlanningToolboxOptimizationSettings optimizationSettings;
   private SolutionQualityConvergenceDetector solutionQualityConvergenceDetector;

   private final YoBoolean initializationSucceeded;
   private final YoInteger numberOfIterations;
   private final YoDouble finalSolutionQuality;

   private SimulationConstructionSet scs;
   private BlockingSimulationRunner blockingSimulationRunner;

   private final HumanoidFloatingRootJointRobot robot;
   private final HumanoidFloatingRootJointRobot ghost;
   private final RobotController toolboxUpdater;

   public HumanoidStepReachabilityCalculator() throws Exception
   {
      mainRegistry = new YoRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      optimizationSettings = new KinematicsPlanningToolboxOptimizationSettings();
      solutionQualityConvergenceDetector = new SolutionQualityConvergenceDetector(optimizationSettings, mainRegistry);

      DRCRobotModel robotModel = getRobotModel();
      DRCRobotModel ghostRobotModel = getRobotModel();

      imposeJointLimitRestrictions(robotModel);

      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());

      double updateDT = 1.0e-3;
      toolboxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                  statusOutputManager,
                                                                  desiredFullRobotModel,
                                                                  updateDT,
                                                                  yoGraphicsListRegistry,
                                                                  mainRegistry);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel, toolboxController.getDesiredReferenceFrames()));
      toolboxController.setInitialRobotConfiguration(robotModel);

      RobotCollisionModel collisionModel = getRobotCollisionModel(robotModel.getJointMap());
      toolboxController.setCollisionModel(collisionModel);

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      // Green collision body
      addKinematicsCollisionGraphics(desiredFullRobotModel, robot, collisionModel);

      // Yellow initial body
      RobotDefinition robotDefinition = ghostRobotModel.getRobotDefinition();
      robotDefinition.setName("Ghost");
      RobotDefinitionTools.setRobotDefinitionMaterial(robotDefinition, ghostMaterial);
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

      switch (mode)
      {
         case HAND_POSE:
            testHandPose();
            break;

         case TEST_SINGLE_STEP:
            StepReachabilityLatticePoint leftFoot = new StepReachabilityLatticePoint(0, 1, 0, 0);
            FramePose3D rightFoot = new FramePose3D();
            testSingleStep(leftFoot, rightFoot, false);
            break;

         case TEST_MULTIPLE_STEPS:
            List<StepReachabilityLatticePoint> leftFootPoseList = createLeftFootPoseList();
            FramePose3D rightFootPose = new FramePose3D();
            List<KinematicsToolboxSnapshotDescription> snapshotDescriptions = new ArrayList<>();

            // Loop through XYZs, free yaw
            for (int xyzLoopIndex = 0; xyzLoopIndex < leftFootPoseList.size(); xyzLoopIndex++)
            {
               StepReachabilityLatticePoint leftFootPose = leftFootPoseList.get(xyzLoopIndex);
               KinematicsToolboxSnapshotDescription snapshotDescription = testSingleStep(leftFootPose, rightFootPose, true);

               double solutionQuality = snapshotDescription.getIkSolution().getSolutionQuality();
               LogTools.info("Reachability value: " + solutionQuality);

               // If there is a valid configuration, sweep through yaws for that XYZ pose
               if (solutionQuality < SOLUTION_QUALITY_THRESHOLD)
               {
                  LogTools.info("Entering yaw sweep");
                  List<StepReachabilityLatticePoint> leftFootYawSweepList = createLeftFootYawSweepList(leftFootPose);

                  for (int yawLoopIndex = 0; yawLoopIndex < leftFootYawSweepList.size(); yawLoopIndex++)
                  {
                     leftFootPose = leftFootYawSweepList.get(yawLoopIndex);
                     snapshotDescription = testSingleStep(leftFootPose, rightFootPose, false);
                     snapshotDescriptions.add(snapshotDescription);
                  }
               }
            }

            String filePath = getResourcesDirectory().replace('/', File.separatorChar) + File.separator + robotModel.getStepReachabilityResourceName()
                                                                                                                    .replace('/', File.separatorChar);
            File reachabilityFile = new File(filePath);
            StepReachabilityIOHelper.writeToFile(reachabilityFile, snapshotDescriptions, spacingXYZ, yawDivisions, maximumOffsetYaw - minimumOffsetYaw);

            break;

         case TEST_VISUALIZATION:
            StepReachabilityData stepReachabilityData = robotModel.getStepReachabilityData();
            new StepReachabilityVisualizer(stepReachabilityData);
            break;

         case TEST_WRITE_SCRIPT:
            filePath = getResourcesDirectory().replace('/', File.separatorChar) + File.separator + robotModel.getStepReachabilityResourceName()
                                                                                                             .replace('/', File.separatorChar);
            reachabilityFile = new File(filePath);

            snapshotDescriptions = new ArrayList<>();
            snapshotDescriptions.add(testSingleStep(new StepReachabilityLatticePoint(0, 2, 0, 0), new FramePose3D(), false));
            StepReachabilityIOHelper.writeToFile(reachabilityFile, snapshotDescriptions, spacingXYZ, yawDivisions, maximumOffsetYaw - minimumOffsetYaw);
            break;

         case TEST_LOAD_SCRIPT:
            StepReachabilityIOHelper stepReachabilityIOHelper = new StepReachabilityIOHelper();
            stepReachabilityData = stepReachabilityIOHelper.loadStepReachability(getRobotModel());

            LogTools.info("Number of kinematic snapshots: " + stepReachabilityIOHelper.getReachabilityIKData().size());
            LogTools.info("spacingXYZ: " + stepReachabilityData.getXyzSpacing());
            LogTools.info("gridSizeYaw: " + stepReachabilityData.getGridSizeYaw());
            LogTools.info("yawDivisions: " + stepReachabilityData.getYawDivisions());
            break;

         default:
            throw new RuntimeException(mode + " is not implemented yet!");
      }

      ThreadTools.sleepForever();
   }

   protected abstract DRCRobotModel getRobotModel();

   protected abstract String getResourcesDirectory();

   protected void imposeJointLimitRestrictions(DRCRobotModel robotModel)
   {
   }

   protected abstract RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap);

   private void testHandPose() throws Exception
   {
      LogTools.info("Entering: testHandPose");
      Random random = new Random(2134);
      double groundHeight = EuclidCoreRandomTools.nextDouble(random, 0.1);
      Point2D offset = EuclidCoreRandomTools.nextPoint2D(random, 2.0);
      double offsetYaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);

      double averageSolutionQuality = 0.0;
      double worstSolutionQuality = -1.0;

      int numberOfTests = 30;

      for (int i = 0; i < numberOfTests; i++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            randomizeArmJointPositions(random, robotSide, randomizedFullRobotModel, 0.4);

            // change to holdRigidBodyAtSomePose
            KinematicsToolboxRigidBodyMessage message = holdRigidBodyCurrentPose(randomizedFullRobotModel.getHand(robotSide));
            message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
            commandInputManager.submitMessage(message);
         }

         { // Setup CoM message
            KinematicsToolboxCenterOfMassMessage message = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(
                  randomizedFullRobotModel));
            SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
            selectionMatrix.selectZAxis(false);
            message.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix));
            message.getWeights().set(MessageTools.createWeightMatrix3DMessage(COM_WEIGHT));
            commandInputManager.submitMessage(message);
         }

         // Disable the support polygon constraint, the randomized model isn't constrained.
         KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
         configurationMessage.setDisableSupportPolygonConstraint(true);
         configurationMessage.setEnableCollisionAvoidance(true);
         commandInputManager.submitMessage(configurationMessage);

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(robotConfigurationData));

         // holds the feet at current configuration
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, true));

         runKinematicsToolboxController();

         if (!initializationSucceeded.getBooleanValue())
         {
            throw new RuntimeException(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         }

         double solutionQuality = toolboxController.getSolution().getSolutionQuality();

         LogTools.info("Solution quality: " + solutionQuality);
         averageSolutionQuality += solutionQuality / numberOfTests;
         worstSolutionQuality = Math.max(worstSolutionQuality, solutionQuality);
      }
   }

   /**
    * Solves IK assuming that the right foot is at the origin and left foot is at the given position
    */
   private KinematicsToolboxSnapshotDescription testSingleStep(StepReachabilityLatticePoint leftFootLatticePoint, FramePose3D rightFoot, boolean freeYaw)
         throws SimulationExceededMaximumTimeException
   {
      if (freeYaw)
         LogTools.info("Entering: testStep");
      else
         LogTools.info("Sweeping yaw");

      double groundHeight = 0.0;
      Point2D offset = new Point2D(0.0, 0.15);
      double offsetYaw = 0.0;

      FullHumanoidRobotModel targetFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(targetFullRobotModel);
      KinematicsToolboxSnapshotDescription snapshotDescription = new KinematicsToolboxSnapshotDescription();
      List<SixDoFMotionControlAnchorDescription> sixDoFMotionControlAnchorDescriptions = new ArrayList<>();
      List<OneDoFMotionControlAnchorDescription> oneDoFMotionControlAnchorDescriptions = new ArrayList<>();

      FramePose3D leftFoot = new FramePose3D();
      leftFoot.getPosition().setX(spacingXYZ * leftFootLatticePoint.getXIndex());
      leftFoot.getPosition().setY(spacingXYZ * leftFootLatticePoint.getYIndex());
      leftFoot.getPosition().setZ(spacingXYZ * leftFootLatticePoint.getZIndex());
      leftFoot.getOrientation().setToYawOrientation(yawSpacing * leftFootLatticePoint.getYawIndex());

      for (RobotSide robotSide : RobotSide.values)
      {
         // Rigid body objective for each foot
         KinematicsToolboxRigidBodyMessage footObjective;
         if (freeYaw)
         {
            if (robotSide == RobotSide.LEFT)
               footObjective = holdRigidBodyFreeYaw(targetFullRobotModel.getFoot(robotSide), leftFoot);
            else
               footObjective = holdRigidBodyFreeYaw(targetFullRobotModel.getFoot(robotSide), rightFoot);
         }
         else
         {
            if (robotSide == RobotSide.LEFT)
               footObjective = holdRigidBodyAtTargetFrame(targetFullRobotModel.getFoot(robotSide), leftFoot);
            else
               footObjective = holdRigidBodyAtTargetFrame(targetFullRobotModel.getFoot(robotSide), rightFoot);
         }

         footObjective.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_FEET_WEIGHT));
         footObjective.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_FEET_WEIGHT));
         commandInputManager.submitMessage(footObjective);
         sixDoFMotionControlAnchorDescriptions.add(sixDoFMessageToDescription(targetFullRobotModel.getFoot(robotSide), footObjective));

         // OneDoFJoint objective for each knee joint TODO Find a way to prevent straight legs
         //         OneDoFJoint kneeJoint = (OneDoFJoint) targetFullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);
         //         KinematicsToolboxOneDoFJointMessage jointMessage = newOneDoFJointMessage(kneeJoint, JOINT_WEIGHT, 1.04);
         //         commandInputManager.submitMessage(jointMessage);
      }

      FramePose3D centerFeet = interpolateFeet(leftFoot, rightFoot);

      // Rigid body objective for chest, center XY pose of feet
      KinematicsToolboxRigidBodyMessage chestObjective = holdRigidBodyAtTargetFrame(targetFullRobotModel.getChest(), centerFeet);
      chestObjective.getLinearSelectionMatrix().setZSelected(false);
      chestObjective.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
      chestObjective.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
      commandInputManager.submitMessage(chestObjective);
      sixDoFMotionControlAnchorDescriptions.add(sixDoFMessageToDescription(targetFullRobotModel.getChest(), chestObjective));

      // Rigid body objective for head, center XY pose of feet
      KinematicsToolboxRigidBodyMessage headObjective = holdRigidBodyAtTargetFrame(targetFullRobotModel.getHead(), centerFeet);
      headObjective.getLinearSelectionMatrix().setZSelected(false);
      headObjective.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
      headObjective.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
      commandInputManager.submitMessage(headObjective);
      sixDoFMotionControlAnchorDescriptions.add(sixDoFMessageToDescription(targetFullRobotModel.getHead(), headObjective));

      { // Setup CoM message
         KinematicsToolboxCenterOfMassMessage comMessage = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCoMForFeet(leftFoot, rightFoot));
         SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
         selectionMatrix.selectZAxis(false);
         comMessage.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix));
         comMessage.getWeights().set(MessageTools.createWeightMatrix3DMessage(COM_WEIGHT));
         commandInputManager.submitMessage(comMessage);
         snapshotDescription.setCenterOfMassAnchor(centerOfMassMessageToDescription(comMessage));
      }

      // Disable the support polygon constraint, the randomized model isn't constrained.
      KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
      configurationMessage.setDisableSupportPolygonConstraint(true);
      configurationMessage.setEnableCollisionAvoidance(true);
      commandInputManager.submitMessage(configurationMessage);

      //      snapGhostToFullRobotModel(targetFullRobotModel);
      toolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(robotConfigurationData));

      runKinematicsToolboxController();

      if (!initializationSucceeded.getBooleanValue())
      {
         throw new RuntimeException(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
      }

      // populate empty fields for these values
      KinematicsToolboxOutputStatus kinematicsSolution = new KinematicsToolboxOutputStatus(toolboxController.getSolution());
      snapshotDescription.setIkSolution(kinematicsSolution);
      snapshotDescription.setControllerConfiguration(new RobotConfigurationData());
      snapshotDescription.setIkPrivilegedConfiguration(new KinematicsToolboxPrivilegedConfigurationMessage());
      snapshotDescription.setOneDoFAnchors(oneDoFMotionControlAnchorDescriptions);
      snapshotDescription.setSixDoFAnchors(sixDoFMotionControlAnchorDescriptions);

      return snapshotDescription;
   }

   private void runKinematicsToolboxController() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      // Using convergence detector currently very slow
      // Can play around with convergence settings to speed things up
      solutionQualityConvergenceDetector.initialize();
      if (visualize)
      {
         while (!solutionQualityConvergenceDetector.isSolved())
         {
            blockingSimulationRunner.simulateNTicksAndBlockAndCatchExceptions(1);
            solutionQualityConvergenceDetector.submitSolutionQuality(toolboxController.getSolution().getSolutionQuality());
            solutionQualityConvergenceDetector.update();
         }
      }
      else
      {
         while (!solutionQualityConvergenceDetector.isSolved())
         {
            toolboxUpdater.doControl();
            solutionQualityConvergenceDetector.submitSolutionQuality(toolboxController.getSolution().getSolutionQuality());
            solutionQualityConvergenceDetector.update();
         }
      }
      finalSolutionQuality.set(toolboxController.getSolution().getSolutionQuality());
   }

   private static final double minimumOffsetX = 0.0;
   private static final double maximumOffsetX = 0.3;
   private static final double minimumOffsetY = 0.0;
   private static final double maximumOffsetY = 0.3;
   private static final double minimumOffsetZ = 0.0;
   private static final double maximumOffsetZ = 0.3;
   private static final double minimumOffsetYaw = -Math.toRadians(70.0);
   private static final double maximumOffsetYaw = Math.toRadians(80.0);

   private static final double spacingXYZ = 0.2; // 0.05
   private static final int yawDivisions = 2;   // 10
   private static final double yawSpacing = (maximumOffsetYaw - minimumOffsetYaw) / yawDivisions;

   private static List<StepReachabilityLatticePoint> createLeftFootPoseList()
   {
      List<StepReachabilityLatticePoint> posesToCheck = new ArrayList<>();

      int minimumXIndex = (int) Math.round(minimumOffsetX / spacingXYZ);
      int maximumXIndex = (int) Math.round(maximumOffsetX / spacingXYZ);
      int minimumYIndex = (int) Math.round(minimumOffsetY / spacingXYZ);
      int maximumYIndex = (int) Math.round(maximumOffsetY / spacingXYZ);
      int minimumZIndex = (int) Math.round(minimumOffsetZ / spacingXYZ);
      int maximumZIndex = (int) Math.round(maximumOffsetZ / spacingXYZ);

      for (int xIndex = minimumXIndex; xIndex <= maximumXIndex; xIndex++)
      {
         for (int yIndex = minimumYIndex; yIndex <= maximumYIndex; yIndex++)
         {
            for (int zIndex = minimumZIndex; zIndex <= maximumZIndex; zIndex++)
            {
               if (xIndex == 0 && yIndex == 0)
                  continue;

               StepReachabilityLatticePoint latticePoint = new StepReachabilityLatticePoint(xIndex, yIndex, zIndex, 0);
               posesToCheck.add(latticePoint);
            }
         }
      }
      return posesToCheck;
   }

   private static List<StepReachabilityLatticePoint> createLeftFootYawSweepList(StepReachabilityLatticePoint leftFootPose)
   {
      List<StepReachabilityLatticePoint> leftFootYawSweepList = new ArrayList<>();

      int minimumYawIndex = -Math.floorMod((int) (Math.round((minimumOffsetYaw) / yawSpacing)), yawDivisions);
      int maximumYawIndex = Math.floorMod((int) (Math.round((maximumOffsetYaw) / yawSpacing)), yawDivisions);

      for (int yawIndex = minimumYawIndex; yawIndex <= maximumYawIndex; yawIndex++)
      {
         StepReachabilityLatticePoint latticePoint = new StepReachabilityLatticePoint(leftFootPose.getXIndex(),
                                                                                      leftFootPose.getYIndex(),
                                                                                      leftFootPose.getZIndex(),
                                                                                      yawIndex);
         leftFootYawSweepList.add(latticePoint);
      }

      return leftFootYawSweepList;
   }

   public static void recursivelyModifyGraphics(JointDescription joint, AppearanceDefinition ghostAppearance)
   {
      if (joint == null)
         return;
      LinkDescription link = joint.getLink();
      if (link == null)
         return;
      LinkGraphicsDescription linkGraphics = link.getLinkGraphics();

      if (linkGraphics != null)
      {
         List<Graphics3DPrimitiveInstruction> graphics3dInstructions = linkGraphics.getGraphics3DInstructions();

         if (graphics3dInstructions == null)
            return;

         for (Graphics3DPrimitiveInstruction primitive : graphics3dInstructions)
         {
            if (primitive instanceof Graphics3DInstruction)
            {
               Graphics3DInstruction modelInstruction = (Graphics3DInstruction) primitive;
               modelInstruction.setAppearance(ghostAppearance);
            }
         }
      }

      if (joint.getChildrenJoints() == null)
         return;

      for (JointDescription child : joint.getChildrenJoints())
      {
         recursivelyModifyGraphics(child, ghostAppearance);
      }
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
            {
               initializationSucceeded.set(toolboxController.initialize());
               if (initializationSucceeded.getValue())
               { // Finish this tick so the robot state after initialization can be seen in SCS.
                  jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
                  return;
               }
            }

            if (initializationSucceeded.getBooleanValue())
            {
               toolboxController.updateInternal();
               jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
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

         @Override
         public String getDescription()
         {
            return null;
         }
      };
   }

   private void hideGhost()
   {
      ghost.setPositionInWorld(new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
   }

   private void snapGhostToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      new JointAnglesWriter(ghost, fullHumanoidRobotModel).updateRobotConfigurationBasedOnFullRobotModel();
   }

   public static FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration(DRCRobotModel robotModel)
   {
      return createFullRobotModelAtInitialConfiguration(robotModel, 0.0, 0.0);
   }

   public static FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration(DRCRobotModel robotModel, double groundHeight, double offsetYaw)
   {
      return createFullRobotModelAtInitialConfiguration(robotModel, groundHeight, new Point2D(), offsetYaw);
   }

   public static FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration(DRCRobotModel robotModel,
                                                                                   double groundHeight,
                                                                                   Tuple2DReadOnly offset,
                                                                                   double offsetYaw)
   {
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(groundHeight, offsetYaw).initializeRobot(robot);
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null);
      SensorDataContext sensorDataContext = new SensorDataContext();
      long timestamp = drcPerfectSensorReaderFactory.getSensorReader().read(sensorDataContext);
      drcPerfectSensorReaderFactory.getSensorReader().compute(timestamp, sensorDataContext);
      initialFullRobotModel.getRootJoint().getJointPose().prependTranslation(offset.getX(), offset.getY(), 0.0);
      initialFullRobotModel.updateFrames();
      return initialFullRobotModel;
   }

   public static SideDependentList<ContactablePlaneBody> extractContactableFeet(FullHumanoidRobotModel robotModel,
                                                                                RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      ContactableBodiesFactory<RobotSide> factory = new ContactableBodiesFactory<>();
      factory.setFullRobotModel(robotModel);
      factory.setReferenceFrames(new HumanoidReferenceFrames(robotModel));
      factory.setFootContactPoints(contactPointParameters.getControllerFootGroundContactPoints());
      return new SideDependentList<>(factory.createFootContactablePlaneBodies());
   }

   private static SixDoFMotionControlAnchorDescription sixDoFMessageToDescription(RigidBodyBasics rigidBody, KinematicsToolboxRigidBodyMessage rigidBodyMessage)
   {
      SixDoFMotionControlAnchorDescription anchorDescription = new SixDoFMotionControlAnchorDescription();
      anchorDescription.setRigidBodyName(rigidBody.getName());
      anchorDescription.setInputMessage(new KinematicsToolboxRigidBodyMessage(rigidBodyMessage));
      return anchorDescription;
   }

   private OneDoFMotionControlAnchorDescription oneDoFMessageToDescription(String jointName, KinematicsToolboxOneDoFJointMessage jointMessage)
   {
      OneDoFMotionControlAnchorDescription jointDescription = new OneDoFMotionControlAnchorDescription();
      jointDescription.setJointName(jointName);
      jointDescription.setInputMessage(new KinematicsToolboxOneDoFJointMessage(jointMessage));
      return jointDescription;
   }

   public static ConvexPolygon2D shrinkPolygon(ConvexPolygon2DReadOnly polygonToShrink, double distance)
   {
      ConvexPolygon2D shrunkPolygon = new ConvexPolygon2D();
      new ConvexPolygonScaler().scaleConvexPolygon(polygonToShrink, distance, shrunkPolygon);
      return shrunkPolygon;
   }

   public static Point2D generateRandomPoint2DInPolygon(Random random, ConvexPolygon2DReadOnly polygon)
   {
      int edgeIndex = random.nextInt(polygon.getNumberOfVertices());
      Point2DReadOnly a = polygon.getCentroid();
      Point2DReadOnly b = polygon.getVertex(edgeIndex);
      Point2DReadOnly c = polygon.getNextVertex(edgeIndex);
      return EuclidGeometryRandomTools.nextPoint2DInTriangle(random, a, b, c);
   }

   public static void randomizeArmJointPositions(Random random, RobotSide robotSide, FullHumanoidRobotModel robotModelToModify)
   {
      randomizeArmJointPositions(random, robotSide, robotModelToModify, 1.0);
   }

   public static void randomizeArmJointPositions(Random random,
                                                 RobotSide robotSide,
                                                 FullHumanoidRobotModel robotModelToModify,
                                                 double percentOfMotionRangeAllowed)
   {
      RigidBodyBasics chest = robotModelToModify.getChest();
      RigidBodyBasics hand = robotModelToModify.getHand(robotSide);
      randomizeKinematicsChainPositions(random, chest, hand, percentOfMotionRangeAllowed);
   }

   public static void randomizeKinematicsChainPositions(Random random, RigidBodyBasics base, RigidBodyBasics body)
   {
      randomizeKinematicsChainPositions(random, base, body, 1.0);
   }

   public static void randomizeKinematicsChainPositions(Random random, RigidBodyBasics base, RigidBodyBasics body, double percentOfMotionRangeAllowed)
   {
      percentOfMotionRangeAllowed = MathTools.clamp(percentOfMotionRangeAllowed, 0.0, 1.0);

      OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(base, body);

      randomizeJointPositions(random, joints, percentOfMotionRangeAllowed);
   }

   public static void randomizeJointPositions(Random random, OneDoFJointBasics[] joints, double percentOfMotionRangeAllowed)
   {
      for (OneDoFJointBasics joint : joints)
      {
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();
         double rangeReduction = (1.0 - percentOfMotionRangeAllowed) * (jointLimitUpper - jointLimitLower);
         jointLimitLower += 0.5 * rangeReduction;
         jointLimitUpper -= 0.5 * rangeReduction;
         joint.setQ(RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper));
      }
      MultiBodySystemTools.getRootBody(joints[0].getPredecessor()).updateFramesRecursively();
   }

   private static CenterOfMassMotionControlAnchorDescription centerOfMassMessageToDescription(KinematicsToolboxCenterOfMassMessage centerOfMassMessage)
   {
      CenterOfMassMotionControlAnchorDescription centerOfMassDescription = new CenterOfMassMotionControlAnchorDescription();
      centerOfMassDescription.setInputMessage(centerOfMassMessage);
      return centerOfMassDescription;
   }

   // CoM at elevator
   public static FramePoint3D computeCenterOfMass3D(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      return new FramePoint3D(new CenterOfMassCalculator(fullHumanoidRobotModel.getElevator(), worldFrame).getCenterOfMass());
   }

   private FramePose3D interpolateFeet(FramePose3D leftFoot, FramePose3D rightFoot)
   {
      // 50% interpolation between leftFoot and rightFoot
      double interpolationAlpha = 0.5;
      FramePose3D centerFeet = new FramePose3D();
      centerFeet.interpolate(leftFoot, rightFoot, interpolationAlpha);
      return centerFeet;
   }

   private FramePoint3D computeCoMForFeet(FramePose3D leftFoot, FramePose3D rightFoot)
   {
      // 50% interpolation between feet, at chest height
      FramePose3D centerPose = interpolateFeet(leftFoot, rightFoot);
      centerPose.getPosition().setZ(1.3496);

      return new FramePoint3D(centerPose.getPosition());
   }

   public static RobotConfigurationData extractRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      fullRobotModel.updateFrames();
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints, forceSensorDefinitions, imuDefinitions);
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));
      robotConfigurationData.getRootPosition().set(fullRobotModel.getRootJoint().getJointPose().getPosition());
      robotConfigurationData.getRootOrientation().set(fullRobotModel.getRootJoint().getJointPose().getOrientation());
      return robotConfigurationData;
   }

   public static CapturabilityBasedStatus createCapturabilityBasedStatus(FullHumanoidRobotModel currentRobotModel,
                                                                         DRCRobotModel drcRobotModel,
                                                                         boolean isLeftFootInSupport,
                                                                         boolean isRightFootInSupport)
   {
      return createCapturabilityBasedStatus(currentRobotModel, drcRobotModel.getContactPointParameters(), isLeftFootInSupport, isRightFootInSupport);
   }

   public static CapturabilityBasedStatus createCapturabilityBasedStatus(FullHumanoidRobotModel currentRobotModel,
                                                                         RobotContactPointParameters<RobotSide> contactPointParameters,
                                                                         boolean isLeftFootInSupport,
                                                                         boolean isRightFootInSupport)
   {
      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      SideDependentList<ContactablePlaneBody> contactableFeet = extractContactableFeet(currentRobotModel, contactPointParameters);

      IDLSequence.Object<Point3D> leftFootSupportPolygon2d = capturabilityBasedStatus.getLeftFootSupportPolygon3d();
      IDLSequence.Object<Point3D> rightFootSupportPolygon2d = capturabilityBasedStatus.getRightFootSupportPolygon3d();
      if (isLeftFootInSupport)
         contactableFeet.get(RobotSide.LEFT)
                        .getContactPointsCopy()
                        .stream()
                        .peek(cp -> cp.changeFrame(worldFrame))
                        .forEach(cp -> leftFootSupportPolygon2d.add().set(cp.getX(), cp.getY(), 0.0));
      if (isRightFootInSupport)
         contactableFeet.get(RobotSide.RIGHT)
                        .getContactPointsCopy()
                        .stream()
                        .peek(cp -> cp.changeFrame(worldFrame))
                        .forEach(cp -> rightFootSupportPolygon2d.add().set(cp.getX(), cp.getY(), 0.0));
      return capturabilityBasedStatus;
   }

   private static Graphics3DObject getGraphics(Collidable collidable)
   {
      Shape3DReadOnly shape = collidable.getShape();
      RigidBodyTransform transformToParentJoint = collidable.getShape()
                                                            .getReferenceFrame()
                                                            .getTransformToDesiredFrame(collidable.getRigidBody().getParentJoint().getFrameAfterJoint());
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.transform(transformToParentJoint);
      AppearanceDefinition appearance = YoAppearance.DarkGreen();
      appearance.setTransparency(0.5);

      if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         graphics.translate(sphere.getPosition());
         graphics.addSphere(sphere.getRadius(), appearance);
      }
      else if (shape instanceof Capsule3DReadOnly)
      {
         Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
         RigidBodyTransform transform = new RigidBodyTransform();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule.getAxis(), transform.getRotation());
         transform.getTranslation().set(capsule.getPosition());
         graphics.transform(transform);
         graphics.addCapsule(capsule.getRadius(), capsule.getLength() + 2.0 * capsule.getRadius(), // the 2nd term is removed internally.
                             appearance);
      }
      else if (shape instanceof Box3DReadOnly)
      {
         Box3DReadOnly box = (Box3DReadOnly) shape;
         graphics.translate(box.getPosition());
         graphics.rotate(new RotationMatrix(box.getOrientation()));
         graphics.addCube(box.getSizeX(), box.getSizeY(), box.getSizeZ(), true, appearance);
      }
      else if (shape instanceof PointShape3DReadOnly)
      {
         PointShape3DReadOnly pointShape = (PointShape3DReadOnly) shape;
         graphics.translate(pointShape);
         graphics.addSphere(0.01, appearance);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape.getClass().getSimpleName());
      }
      return graphics;
   }

   public static void addKinematicsCollisionGraphics(FullHumanoidRobotModel fullRobotModel, Robot robot, RobotCollisionModel collisionModel)
   {
      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(fullRobotModel.getElevator());

      for (Collidable collidable : robotCollidables)
      {
         Graphics3DObject linkGraphics = robot.getLink(collidable.getRigidBody().getName()).getLinkGraphics();

         if (linkGraphics != null)
            linkGraphics.combine(getGraphics(collidable));
      }
   }
}
