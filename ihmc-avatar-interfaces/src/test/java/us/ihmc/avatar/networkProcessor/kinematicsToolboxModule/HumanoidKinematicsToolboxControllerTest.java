package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.IKRobotStateUpdater;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.*;
import java.util.stream.Collectors;

import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;
import static org.junit.jupiter.api.Assertions.assertTrue;

@Tag("humanoid-toolbox")
public abstract class HumanoidKinematicsToolboxControllerTest implements MultiRobotTestInterface
{
   private static final boolean VERBOSE = false;

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
   private HumanoidKinematicsToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;
   private YoDouble finalSolutionQuality;

   private SimulationConstructionSet2 scs;

   private Robot robot;
   private Robot ghost;
   private Controller toolboxUpdater;

   /**
    * Returns a separate instance of the robot model that will be modified in this test to create a
    * ghost robot.
    */
   public abstract DRCRobotModel getGhostRobotModel();

   public void setup()
   {
      setup(false);
   }

   public void setup(boolean setupForMultiContact)
   {
      mainRegistry = new YoRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();

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

      if (!setupForMultiContact)
         toolboxController.setInitialRobotConfiguration(robotModel);

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
      if (visualize && simulationTestingParameters.getKeepSCSUp())
      {
         scs.startSimulationThread();
         scs.waitUntilVisualizerDown();
      }

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
   }

   @Test
   public void testHoldBodyPose() throws Exception
   {
      setup();

      Random random = new Random(4576488);
      double groundHeight = EuclidCoreRandomTools.nextDouble(random, 0.1);
      Point2D offset = EuclidCoreRandomTools.nextPoint2D(random, 2.0);
      double offsetYaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);
      snapGhostToFullRobotModel(initialFullRobotModel);

      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getPelvis()));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getChest()));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getHand(RobotSide.LEFT)));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getHand(RobotSide.RIGHT)));

      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(robotConfigurationData));
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(initialFullRobotModel, getRobotModel(), true, true));

      int numberOfIterations = 250;

      runKinematicsToolboxController(numberOfIterations);

      assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
      assertTrue(toolboxController.getSolution().getSolutionQuality() < 1.0e-4, "Poor solution quality: " + toolboxController.getSolution().getSolutionQuality());
   }

   @Test
   public void testRandomHandPositions() throws Exception
   {
      setup();

      if (VERBOSE)
         LogTools.info("Entering: testRandomHandPositions");
      Random random = new Random(2135);
      double groundHeight = EuclidCoreRandomTools.nextDouble(random, 0.1);
      Point2D offset = EuclidCoreRandomTools.nextPoint2D(random, 2.0);
      double offsetYaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);

      for (int i = 0; i < 10; i++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            randomizeArmJointPositions(random, robotSide, randomizedFullRobotModel, 0.6);
            RigidBodyBasics hand = randomizedFullRobotModel.getHand(robotSide);
            FramePoint3D desiredPosition = new FramePoint3D(hand.getBodyFixedFrame());
            desiredPosition.changeFrame(worldFrame);
            KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, desiredPosition);
            message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
            commandInputManager.submitMessage(message);
         }

         { // Setup CoM message
            KinematicsToolboxCenterOfMassMessage message = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(
                  randomizedFullRobotModel));
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
         toolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(robotConfigurationData));
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 100;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            LogTools.info("Solution quality: " + solutionQuality);
         assertTrue(solutionQuality < 3.0e-3, "Poor solution quality: " + solutionQuality);
      }
   }

   @Test
   public void testRandomHandPoses() throws Exception
   {
      setup();

      if (VERBOSE)
         LogTools.info("Entering: testRandomHandPoses");
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
            KinematicsToolboxCenterOfMassMessage message = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(
                  randomizedFullRobotModel));
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
         toolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(robotConfigurationData));
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 150;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            LogTools.info("Solution quality: " + solutionQuality);
         averageSolutionQuality += solutionQuality / numberOfTests;
         worstSolutionQuality = Math.max(worstSolutionQuality, solutionQuality);
      }

      if (VERBOSE)
      {
         LogTools.info("Solution quality: average = " + averageSolutionQuality + ", worst = " + worstSolutionQuality);
      }
      assertTrue(worstSolutionQuality < 5.0e-3, "Poor worst solution quality: " + worstSolutionQuality);
      assertTrue(averageSolutionQuality < 1.0e-3, "Poor average solution quality: " + averageSolutionQuality);
   }

   @Test
   public void testSingleSupport() throws Exception
   {
      setup();

      if (VERBOSE)
         LogTools.info("Entering: testSingleSupport");
      Random random = new Random(2133);
      double groundHeight = EuclidCoreRandomTools.nextDouble(random, 0.1);
      Point2D offset = EuclidCoreRandomTools.nextPoint2D(random, 2.0);
      double offsetYaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);
      RobotSide supportFootSide = RobotSide.LEFT;

      double averageSolutionQuality = 0.0;
      double worstSolutionQuality = -1.0;

      int numberOfTests = 30;

      RigidBodyBasics[] bodiesToControl = {randomizedFullRobotModel.getChest(),
                                           randomizedFullRobotModel.getHand(RobotSide.LEFT),
                                           randomizedFullRobotModel.getHand(RobotSide.RIGHT),
                                           randomizedFullRobotModel.getFoot(supportFootSide.getOppositeSide())};

      for (int i = 0; i < numberOfTests; i++)
      {
         randomizeJointPositions(random, randomizedFullRobotModel.getOneDoFJoints(), 0.33);
         randomizedFullRobotModel.updateFrames();
         ReferenceFrame rootJointFrame = randomizedFullRobotModel.getRootJoint().getFrameAfterJoint();
         ReferenceFrame supportFootFrame = randomizedFullRobotModel.getFoot(supportFootSide).getBodyFixedFrame();
         RigidBodyTransform transformFromRootJointToWorldFrame = rootJointFrame.getTransformToDesiredFrame(supportFootFrame);
         RigidBodyTransform initialSupportFootTransform = initialFullRobotModel.getFoot(supportFootSide).getBodyFixedFrame().getTransformToWorldFrame();
         transformFromRootJointToWorldFrame.preMultiply(initialSupportFootTransform);

         randomizedFullRobotModel.getRootJoint().setJointConfiguration(transformFromRootJointToWorldFrame);
         randomizedFullRobotModel.updateFrames();

         for (RigidBodyBasics rigidBody : bodiesToControl)
         {
            FramePoint3D desiredPosition = new FramePoint3D(rigidBody.getBodyFixedFrame());
            desiredPosition.changeFrame(worldFrame);
            KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(rigidBody, desiredPosition);
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
            commandInputManager.submitMessage(message);
         }

         { // Setup CoM message
            KinematicsToolboxCenterOfMassMessage message = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(
                  randomizedFullRobotModel));
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
         toolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(robotConfigurationData));
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, false));

         int numberOfIterations = 350;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            LogTools.info("Solution quality: " + solutionQuality);
         averageSolutionQuality += solutionQuality / numberOfTests;
         worstSolutionQuality = Math.max(worstSolutionQuality, solutionQuality);
      }

      if (VERBOSE)
      {
         LogTools.info("Solution quality: average = " + averageSolutionQuality + ", worst = " + worstSolutionQuality);
      }
      assertTrue(worstSolutionQuality < 5.0e-2, "Poor worst solution quality: " + worstSolutionQuality);
      assertTrue(averageSolutionQuality < 6.5e-3, "Poor average solution quality: " + averageSolutionQuality);
   }

   @Test
   public void testCenterOfMassConstraint() throws Exception
   {
      setup();

      if (VERBOSE)
         LogTools.info("Entering: testCenterOfMassConstraint");

      Random random = new Random(21651);

      double groundHeight = EuclidCoreRandomTools.nextDouble(random, 0.1);
      Point2D offset = EuclidCoreRandomTools.nextPoint2D(random, 2.0);
      double offsetYaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, offset, offsetYaw);
      RigidBodyBasics chest = initialFullRobotModel.getChest();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = initialFullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         for (OneDoFJointBasics joint : armJoints)
         {
            joint.setQ(0.5 * (joint.getJointLimitLower() + joint.getJointLimitUpper()));
         }
      }
      initialFullRobotModel.updateFrames();
      toolboxController.setInitialRobotConfigurationNamedMap(newInitialConfigurationMap(initialFullRobotModel));
      snapGhostToFullRobotModel(initialFullRobotModel);

      double comSafeMargin = toolboxController.getCenterOfMassSafeMargin().getValue();
      ConvexPolygon2D supportPolygon = extractSupportPolygon(initialFullRobotModel, getRobotModel().getContactPointParameters());
      ConvexPolygon2D shrunkSupportPolygon = shrinkPolygon(supportPolygon, comSafeMargin);

      for (int i = 0; i < 15; i++)
      { // Asserts that the CoM can move inside the support polygon.
         Point2D randomSafeCoMLocation = generateRandomPoint2DInPolygon(random, shrunkSupportPolygon);

         Vector3D shift = new Vector3D(randomSafeCoMLocation);
         shift.sub(computeCenterOfMass3D(initialFullRobotModel));
         shift.setZ(-0.08);
         commandInputManager.submitMessage(shiftCoMMessage(initialFullRobotModel, shift, 0.1));
         commandInputManager.submitMessage(shiftBodyMessage(initialFullRobotModel.getPelvis(), shift, 5.0, true, false));
         commandInputManager.submitMessage(shiftBodyMessage(chest, shift, 5.0, true, false));
         commandInputManager.submitMessage(shiftBodyMessage(initialFullRobotModel.getHand(RobotSide.LEFT), shift, 5.0));
         commandInputManager.submitMessage(shiftBodyMessage(initialFullRobotModel.getHand(RobotSide.RIGHT), shift, 5.0));

         RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
         toolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(robotConfigurationData));
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(initialFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 250;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         assertTrue(toolboxController.getSolution().getSolutionQuality() < 1.0e-4, "Poor solution quality: " + toolboxController.getSolution().getSolutionQuality());
      }

      for (int i = 0; i < 15; i++)
      { // Asserts that the CoM doesn't move outside the support polygon
         LineSegment2D edge = new LineSegment2D();
         shrunkSupportPolygon.getEdge(random.nextInt(shrunkSupportPolygon.getNumberOfVertices()), edge);
         Point2DBasics randomCoMLocationLimit = edge.pointBetweenEndpointsGivenPercentage(random.nextDouble());
         Vector2DBasics towardOutside = EuclidGeometryTools.perpendicularVector2D(edge.direction(true));
         Point2D randomUnsafeCoMLocation = new Point2D();
         randomUnsafeCoMLocation.scaleAdd(0.10, towardOutside, randomCoMLocationLimit);

         Vector3D shift = new Vector3D(randomCoMLocationLimit);

         shift.sub(computeCenterOfMass3D(initialFullRobotModel));
         shift.setZ(-0.08);
         commandInputManager.submitMessage(shiftCoMMessage(initialFullRobotModel, shift, 0.1));
         commandInputManager.submitMessage(shiftBodyMessage(initialFullRobotModel.getPelvis(), shift, 50.0, true, false));
         commandInputManager.submitMessage(shiftBodyMessage(chest, shift, 50.0, true, false));
         commandInputManager.submitMessage(shiftBodyMessage(initialFullRobotModel.getHand(RobotSide.LEFT), shift, 5.0));
         commandInputManager.submitMessage(shiftBodyMessage(initialFullRobotModel.getHand(RobotSide.RIGHT), shift, 5.0));

         RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
         toolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(robotConfigurationData));
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(initialFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 250;

         runKinematicsToolboxController(numberOfIterations);

         Point2D centerOfMass2D = new Point2D(computeCenterOfMass3D(toolboxController.getDesiredFullRobotModel()));
         assertTrue(shrunkSupportPolygon.isPointInside(centerOfMass2D, 1.0e-7), "Error: " + shrunkSupportPolygon.signedDistance(centerOfMass2D));
      }
   }

   private static Map<String, Double> newInitialConfigurationMap(FullRobotModel robotModel)
   {
      Map<String, Double> map = new HashMap<>();
      for (OneDoFJointBasics joint : robotModel.getOneDoFJoints())
         map.put(joint.getName(), joint.getQ());
      return map;
   }

   private static KinematicsToolboxRigidBodyMessage shiftBodyMessage(RigidBodyBasics body, Tuple3DReadOnly shift, double weight)
   {
      return shiftBodyMessage(body, shift, weight, true, true);
   }

   private static KinematicsToolboxRigidBodyMessage shiftBodyMessage(RigidBodyBasics body,
                                                                     Tuple3DReadOnly shift,
                                                                     double weight,
                                                                     boolean controlAngular,
                                                                     boolean controlLinear)
   {
      KinematicsToolboxRigidBodyMessage message = holdRigidBodyCurrentPose(body);
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(weight));
      message.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(controlLinear, controlLinear, controlLinear));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(weight));
      message.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(controlAngular, controlAngular, controlAngular));
      message.getDesiredPositionInWorld().add(shift);
      return message;
   }

   private static KinematicsToolboxCenterOfMassMessage shiftCoMMessage(FullHumanoidRobotModel robot, Tuple3DReadOnly shift, double weight)
   {
      return shiftCoMMessage(robot, shift, weight, false);
   }

   private static KinematicsToolboxCenterOfMassMessage shiftCoMMessage(FullHumanoidRobotModel robot, Tuple3DReadOnly shift, double weight, boolean zSelected)
   {
      KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage();
      message.getDesiredPositionInWorld().set(computeCenterOfMass3D(robot));
      message.getDesiredPositionInWorld().add(shift);
      message.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, zSelected));
      message.getWeights().set(MessageTools.createWeightMatrix3DMessage(weight));
      return message;
   }

   private static KinematicsToolboxPrivilegedConfigurationMessage createPrivilegedConfigurationFromRobotModel(FullHumanoidRobotModel fullRobotModel,
                                                                                                              double privilegedConfigurationGain,
                                                                                                              double privilegedConfigurationWeight)
   {
      KinematicsToolboxPrivilegedConfigurationMessage privilegedConfigurationMessage = new KinematicsToolboxPrivilegedConfigurationMessage();
      OneDoFJointBasics[] oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         privilegedConfigurationMessage.getPrivilegedJointHashCodes().add(oneDoFJoints[i].hashCode());
         privilegedConfigurationMessage.getPrivilegedJointAngles().add((float) oneDoFJoints[i].getQ());
      }
      privilegedConfigurationMessage.setPrivilegedGain(privilegedConfigurationGain);
      privilegedConfigurationMessage.setPrivilegedWeight(privilegedConfigurationWeight);

      privilegedConfigurationMessage.setPrivilegedGain(privilegedConfigurationGain);
      privilegedConfigurationMessage.setPrivilegedWeight(privilegedConfigurationWeight);
      return privilegedConfigurationMessage;
   }

   private void runKinematicsToolboxController(int numberOfIterations)
   {
      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         scs.simulateNow(numberOfIterations);
      }
      else
      {
         for (int i = 0; i < numberOfIterations; i++)
            toolboxUpdater.doControl();
      }

      finalSolutionQuality.set(toolboxController.getSolution().getSolutionQuality());
   }

   private Controller createToolboxUpdater()
   {
      return new Controller()
      {
         private final List<? extends JointBasics> toolboxJoints = toolboxController.getDesiredRootJoint().subtreeList();

         @Override
         public void doControl()
         {
            if (!initializationSucceeded.getBooleanValue())
            {
               initializationSucceeded.set(toolboxController.initialize());
               if (initializationSucceeded.getValue())
               { // Finish this tick so the robot state after initialization can be seen in SCS.
                  MultiBodySystemTools.copyJointsState(toolboxJoints, robot.getAllJoints(), JointStateType.CONFIGURATION);
                  return;
               }
            }

            if (initializationSucceeded.getBooleanValue())
            {
               toolboxController.updateInternal();
               MultiBodySystemTools.copyJointsState(toolboxJoints, robot.getAllJoints(), JointStateType.CONFIGURATION);
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
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel(false);
      robotModel.getDefaultRobotInitialSetup(groundHeight, offsetYaw).initializeFullRobotModel(initialFullRobotModel);
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

      Object<Point3D> leftFootSupportPolygon2d = capturabilityBasedStatus.getLeftFootSupportPolygon3d();
      Object<Point3D> rightFootSupportPolygon2d = capturabilityBasedStatus.getRightFootSupportPolygon3d();
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
}
