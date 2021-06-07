package us.ihmc.valkyrie.stepReachability;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.KinematicsPlanningToolboxOptimizationSettings;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.SolutionQualityConvergenceDetector;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.reachabilityMap.footstep.StepReachabilityFileTools;
import us.ihmc.avatar.reachabilityMap.footstep.StepReachabilityVisualizer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSimulationCollisionModel;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.List;
import java.util.*;
import java.util.stream.Collectors;

import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyAtTargetFrame;
import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;
import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.newOneDoFJointMessage;

import static us.ihmc.robotics.Assert.assertTrue;

/**
 * Pattern matched off of HumanoidKinematicsToolboxControllerTest. Check there for reference if needed.
 */
public class ValkyrieStepReachabilityCalculator
{

   private enum Mode
   {
      HAND_POSE, TEST_SINGLE_STEP, TEST_MULTIPLE_STEPS, TEST_VISUALIZATION
   }

   private static final Mode mode = Mode.TEST_MULTIPLE_STEPS;

   private static final double COM_WEIGHT = 1.0;
   private static final double RIGID_BODY_FEET_WEIGHT = 40.0;
   private static final double RIGID_BODY_OTHER_WEIGHT = 20.0;
   private static final double JOINT_WEIGHT = 10.0;
   private static final double SOLUTION_QUALITY_THRESHOLD = 5;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final YoAppearanceRGBColor ghostAppearance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
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


   public static void main(String[] args) throws Exception
   {
      new ValkyrieStepReachabilityCalculator();
   }

   public ValkyrieStepReachabilityCalculator() throws Exception
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

      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel));

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());

      double updateDT = 1.0e-3;
      toolboxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                  statusOutputManager,
                                                                  desiredFullRobotModel,
                                                                  robotModel,
                                                                  updateDT,
                                                                  yoGraphicsListRegistry,
                                                                  mainRegistry);
      toolboxController.setInitialRobotConfiguration(robotModel);

      ValkyrieSimulationCollisionModel collisionModel = new ValkyrieSimulationCollisionModel(robotModel.getJointMap());
      collisionModel.setCollidableHelper(new CollidableHelper(), "robot", "ground");
      toolboxController.setCollisionModel(collisionModel);

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      // Green collision body
      addKinematicsCollisionGraphics(desiredFullRobotModel, robot, collisionModel);

      // Yellow initial body
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName("Ghost");
      recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), ghostAppearance);
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

      switch(mode)
      {
         case HAND_POSE:
            testHandPose();
            break;
         case TEST_SINGLE_STEP:
            FramePose3D leftFoot = new FramePose3D();
            FramePose3D rightFoot = new FramePose3D();
            leftFoot.getPosition().set(0.0, 0.5, 0.000);
//            leftFoot.getOrientation().setYawPitchRoll(0.000,  0.000,  0.996);
            testSingleStep(leftFoot, rightFoot, SOLUTION_QUALITY_THRESHOLD);
            break;
         case TEST_MULTIPLE_STEPS:
            List<FramePose3D> leftFootPoseList = createLeftFootPoseList();
            Map<FramePose3D, Double> poseValidityMap = new HashMap<>();
            FramePose3D rightFootPose = new FramePose3D();
            for (int i = 0; i < leftFootPoseList.size(); i++)
            {
               FramePose3D leftFootPose = leftFootPoseList.get(i);
               double reachabilityValue = testSingleStep(leftFootPose, rightFootPose, SOLUTION_QUALITY_THRESHOLD);
               poseValidityMap.put(leftFootPose, reachabilityValue);
            }
//            new StepReachabilityVisualizer(poseValidityMap, queriesPerAxis);
            StepReachabilityFileTools.writeToFile(poseValidityMap);
//            StepReachabilityFileTools.printFeasibilityMap(poseValidityMap);
            break;
         case TEST_VISUALIZATION:
            Map<FramePose3D, Double> feasibilityMap = StepReachabilityFileTools.loadFromFile("StepReachabilityMap.txt");
//            new StepReachabilityVisualizer(feasibilityMap, queriesPerAxis);
            StepReachabilityFileTools.printFeasibilityMap(feasibilityMap);
            break;
         default:
            throw new RuntimeException(mode + " is not implemented yet!");
      }

      ThreadTools.sleepForever();
   }

   private DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

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
            KinematicsToolboxCenterOfMassMessage message = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(randomizedFullRobotModel));
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
         toolboxController.updateRobotConfigurationData(robotConfigurationData);

         // holds the feet at current configuration
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, true));

         runKinematicsToolboxController();

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();

         LogTools.info("Solution quality: " + solutionQuality);
         averageSolutionQuality += solutionQuality / numberOfTests;
         worstSolutionQuality = Math.max(worstSolutionQuality, solutionQuality);
      }
   }

   /**
    * Solves IK assuming that the right foot is at the origin and left foot is at the given position
    */
   private double testSingleStep(FramePose3D leftFoot, FramePose3D rightFoot, double solutionQualityThreshold) throws SimulationExceededMaximumTimeException
   {
      LogTools.info("Entering: testStep");
      double groundHeight = 0.0;
      Point2D offset = new Point2D(0.0, 0.15);
      double offsetYaw = 0.0;

      FullHumanoidRobotModel targetFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(targetFullRobotModel);

      for (RobotSide robotSide : RobotSide.values)
      {
         // Rigid body objective for each foot
         KinematicsToolboxRigidBodyMessage rbMessage;
         if (robotSide == RobotSide.LEFT)
            rbMessage = holdRigidBodyAtTargetFrame(targetFullRobotModel.getFoot(robotSide), leftFoot);
         else
            rbMessage = holdRigidBodyAtTargetFrame(targetFullRobotModel.getFoot(robotSide), rightFoot);
         rbMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_FEET_WEIGHT));
         rbMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_FEET_WEIGHT));
         commandInputManager.submitMessage(rbMessage);

         // OneDoFJoint objective for each knee joint
         OneDoFJoint kneeJoint = (OneDoFJoint) targetFullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);
         KinematicsToolboxOneDoFJointMessage jointMessage = newOneDoFJointMessage(kneeJoint, JOINT_WEIGHT, 1.04);
         commandInputManager.submitMessage(jointMessage);
      }

      FramePose3D centerFeet = interpolateFeet(leftFoot, rightFoot);

      // Rigid body objective for chest, center XY pose of feet
      KinematicsToolboxRigidBodyMessage rbMessage = holdRigidBodyAtTargetFrame(targetFullRobotModel.getChest(), centerFeet);
      rbMessage.getLinearSelectionMatrix().setZSelected(false);
      rbMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
      rbMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
      commandInputManager.submitMessage(rbMessage);

      // Rigid body objective for head, center XY pose of feet
      rbMessage = holdRigidBodyAtTargetFrame(targetFullRobotModel.getHead(), centerFeet);
      rbMessage.getLinearSelectionMatrix().setZSelected(false);
      rbMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
      rbMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(RIGID_BODY_OTHER_WEIGHT));
      commandInputManager.submitMessage(rbMessage);

      { // Setup CoM message
         KinematicsToolboxCenterOfMassMessage comMessage = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCoMForFeet(leftFoot, rightFoot));
         SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
         selectionMatrix.selectZAxis(false);
         comMessage.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix));
         comMessage.getWeights().set(MessageTools.createWeightMatrix3DMessage(COM_WEIGHT));
         commandInputManager.submitMessage(comMessage);
      }

      // Disable the support polygon constraint, the randomized model isn't constrained.
      KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
      configurationMessage.setDisableSupportPolygonConstraint(true);
      configurationMessage.setEnableCollisionAvoidance(true);
      commandInputManager.submitMessage(configurationMessage);

//      snapGhostToFullRobotModel(targetFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      runKinematicsToolboxController();

      assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
      return toolboxController.getSolution().getSolutionQuality();
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

   private static final int queriesPerAxis = 27;
   private static final double minimumOffsetX = -0.7;
   private static final double maximumOffsetX = 0.7;
   private static final double minimumOffsetY = -0.4;
   private static final double maximumOffsetY = 0.9;
   private static final double minimumOffsetYaw = - Math.toRadians(80.0);
   private static final double maximumOffsetYaw = Math.toRadians(80.0);

   private static List<FramePose3D> createLeftFootPoseList()
   {
      List<FramePose3D> posesToCheck = new ArrayList<>();

      for (int i = 0; i < queriesPerAxis; i++)
      {
         for (int j = 0; j < queriesPerAxis; j++)
         {
            for (int k = 0; k < queriesPerAxis; k++)
            {
               double alphaX = ((double) i) / (queriesPerAxis - 1);
               double alphaY = ((double) j) / (queriesPerAxis - 1);
               double alphaYaw = ((double) k) / (queriesPerAxis - 1);

               double x = EuclidCoreTools.interpolate(minimumOffsetX, maximumOffsetX, alphaX);
               double y = EuclidCoreTools.interpolate(minimumOffsetY, maximumOffsetY, alphaY);
               double yaw = AngleTools.interpolateAngle(minimumOffsetYaw, maximumOffsetYaw, alphaYaw);

               FramePose3D pose = new FramePose3D();
               pose.getPosition().set(x, y, 0.0);
               pose.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);

               // Don't add foot pose where both at origin
               if (pose.getPosition().distanceFromOrigin() != 0)
                  posesToCheck.add(pose);
            }
         }
      }
      return posesToCheck;
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
   public static FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration(DRCRobotModel robotModel, double groundHeight, Tuple2DReadOnly offset, double offsetYaw)
   {
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(groundHeight, offsetYaw).initializeRobot(robot, robotModel.getJointMap());
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null);
      SensorDataContext sensorDataContext = new SensorDataContext();
      long timestamp = drcPerfectSensorReaderFactory.getSensorReader().read(sensorDataContext);
      drcPerfectSensorReaderFactory.getSensorReader().compute(timestamp, sensorDataContext);
      initialFullRobotModel.getRootJoint().getJointPose().prependTranslation(offset.getX(), offset.getY(), 0.0);
      initialFullRobotModel.updateFrames();
      return initialFullRobotModel;
   }

   public static ConvexPolygon2D extractSupportPolygon(FullHumanoidRobotModel initialFullRobotModel,
                                                       RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      SideDependentList<ContactablePlaneBody> contactableFeet = extractContactableFeet(initialFullRobotModel, contactPointParameters);
      ConvexPolygon2D supportPolygon = new ConvexPolygon2D();

      for (RobotSide robotSide : RobotSide.values)
      {
         List<FramePoint3D> contactPoints = contactableFeet.get(robotSide).getContactPointsCopy();
         contactPoints.forEach(cp -> cp.changeFrame(worldFrame));
         supportPolygon.addVertices(Vertex3DSupplier.asVertex3DSupplier(contactPoints));
      }
      supportPolygon.update();
      return supportPolygon;
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

   public static void randomizeArmJointPositions(Random random, RobotSide robotSide, FullHumanoidRobotModel robotModelToModify,
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
      robotConfigurationData.getRootTranslation().set(fullRobotModel.getRootJoint().getJointPose().getPosition());
      robotConfigurationData.getRootOrientation().set(fullRobotModel.getRootJoint().getJointPose().getOrientation());
      return robotConfigurationData;
   }

   public static CapturabilityBasedStatus createCapturabilityBasedStatus(FullHumanoidRobotModel currentRobotModel, DRCRobotModel drcRobotModel,
                                                                         boolean isLeftFootInSupport, boolean isRightFootInSupport)
   {
      return createCapturabilityBasedStatus(currentRobotModel, drcRobotModel.getContactPointParameters(), isLeftFootInSupport, isRightFootInSupport);
   }

   public static CapturabilityBasedStatus createCapturabilityBasedStatus(FullHumanoidRobotModel currentRobotModel,
                                                                         RobotContactPointParameters<RobotSide> contactPointParameters,
                                                                         boolean isLeftFootInSupport, boolean isRightFootInSupport)
   {
      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      SideDependentList<ContactablePlaneBody> contactableFeet = extractContactableFeet(currentRobotModel, contactPointParameters);

      IDLSequence.Object<Point3D> leftFootSupportPolygon2d = capturabilityBasedStatus.getLeftFootSupportPolygon3d();
      IDLSequence.Object<Point3D> rightFootSupportPolygon2d = capturabilityBasedStatus.getRightFootSupportPolygon3d();
      if (isLeftFootInSupport)
         contactableFeet.get(RobotSide.LEFT).getContactPointsCopy().stream().peek(cp -> cp.changeFrame(worldFrame))
                        .forEach(cp -> leftFootSupportPolygon2d.add().set(cp.getX(), cp.getY(), 0.0));
      if (isRightFootInSupport)
         contactableFeet.get(RobotSide.RIGHT).getContactPointsCopy().stream().peek(cp -> cp.changeFrame(worldFrame))
                        .forEach(cp -> rightFootSupportPolygon2d.add().set(cp.getX(), cp.getY(), 0.0));
      return capturabilityBasedStatus;
   }

   private static Graphics3DObject getGraphics(Collidable collidable)
   {
      Shape3DReadOnly shape = collidable.getShape();
      RigidBodyTransform transformToParentJoint = collidable.getShape().getReferenceFrame()
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
         graphics.addCapsule(capsule.getRadius(),
                             capsule.getLength() + 2.0 * capsule.getRadius(), // the 2nd term is removed internally.
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
         Link link = robot.getLink(collidable.getRigidBody().getName());
         link.getLinkGraphics().combine(getGraphics(collidable));
      }
   }

}
