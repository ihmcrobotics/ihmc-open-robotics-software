package us.ihmc.valkyrie.stepReachability;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
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
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
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
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;
import static us.ihmc.robotics.Assert.assertTrue;

/**
 * Pattern matched off of HumanoidKinematicsToolboxController. Check there for reference if needed.
 */
public class ValkyrieStepReachabilityCalculator
{
   private enum Mode
   {
      HAND_POSE, TEST_STEP
   }

   private static final Mode mode = Mode.HAND_POSE;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
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

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName("Ghost");
      recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), ghostApperance);
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
         case TEST_STEP:
            FramePose3D leftFoot = new FramePose3D();
            FramePose3D rightFoot = new FramePose3D();
            rightFoot.getPosition().set(0.0, -0.25, 0.0);
            testStep(leftFoot, rightFoot);
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
            KinematicsToolboxRigidBodyMessage message = holdRigidBodyCurrentPose(randomizedFullRobotModel.getHand(robotSide));
            message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
            commandInputManager.submitMessage(message);
         }

         { // Setup CoM message
            KinematicsToolboxCenterOfMassMessage message = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(randomizedFullRobotModel));
            SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
            selectionMatrix.selectZAxis(false);
            message.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix));
            message.getWeights().set(MessageTools.createWeightMatrix3DMessage(1.0));
            commandInputManager.submitMessage(message);
         }

         // Disable the support polygon constraint, the randomized model isn't constrained.
         KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
         configurationMessage.setDisableSupportPolygonConstraint(true);
         commandInputManager.submitMessage(configurationMessage);

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 150;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();

         LogTools.info("Solution quality: " + solutionQuality);
         averageSolutionQuality += solutionQuality / numberOfTests;
         worstSolutionQuality = Math.max(worstSolutionQuality, solutionQuality);
      }
   }

   private void testStep(FramePose3D leftFoot, FramePose3D rightFoot)
   {
      // TODO
   }

   private void runKinematicsToolboxController(int numberOfIterations) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         blockingSimulationRunner.simulateNTicksAndBlockAndCatchExceptions(numberOfIterations);
      }
      else
      {
         for (int i = 0; i < numberOfIterations; i++)
            toolboxUpdater.doControl();
      }

      finalSolutionQuality.set(toolboxController.getSolution().getSolutionQuality());
   }

   public static void recursivelyModifyGraphics(JointDescription joint, AppearanceDefinition ghostApperance)
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
               modelInstruction.setAppearance(ghostApperance);
            }
         }
      }

      if (joint.getChildrenJoints() == null)
         return;

      for (JointDescription child : joint.getChildrenJoints())
      {
         recursivelyModifyGraphics(child, ghostApperance);
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

   public static FramePoint3D computeCenterOfMass3D(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      return new FramePoint3D(new CenterOfMassCalculator(fullHumanoidRobotModel.getElevator(), worldFrame).getCenterOfMass());
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

}
