package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;
import static us.ihmc.robotics.Assert.assertTrue;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
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
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class HumanoidKinematicsToolboxControllerTest implements MultiRobotTestInterface
{
   private static final boolean VERBOSE = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();
   static
   {
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private CommandInputManager commandInputManager;
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private HumanoidKinematicsToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;
   private YoDouble finalSolutionQuality;

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
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();

      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel));

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());

      toolboxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                  statusOutputManager,
                                                                  desiredFullRobotModel,
                                                                  yoGraphicsListRegistry,
                                                                  mainRegistry);

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
   }

   @Test
   public void testHoldBodyPose() throws Exception
   {
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel());
      snapGhostToFullRobotModel(initialFullRobotModel);

      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getPelvis()));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getChest()));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getHand(RobotSide.LEFT)));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getHand(RobotSide.RIGHT)));

      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(initialFullRobotModel, getRobotModel(), true, true));

      int numberOfIterations = 250;

      runKinematicsToolboxController(numberOfIterations);

      assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
      assertTrue("Poor solution quality: " + toolboxController.getSolution().getSolutionQuality(),
                 toolboxController.getSolution().getSolutionQuality() < 1.0e-4);
   }

   @Test
   public void testRandomHandPositions() throws Exception
   {
      if (VERBOSE)
         LogTools.info("Entering: testRandomHandPositions");
      Random random = new Random(2135);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel());
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel());

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
            KinematicsToolboxCenterOfMassMessage message = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(randomizedFullRobotModel));
            SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
            selectionMatrix.selectZAxis(false);
            message.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix));
            message.getWeights().set(MessageTools.createWeightMatrix3DMessage(1.0));
            commandInputManager.submitMessage(message);
         }

         // Disable the support polygon constraint, the randomized model isn't constrained.
         HumanoidKinematicsToolboxConfigurationMessage configurationMessage = new HumanoidKinematicsToolboxConfigurationMessage();
         configurationMessage.setEnableSupportPolygonConstraint(false);
         commandInputManager.submitMessage(configurationMessage);

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 100;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            LogTools.info("Solution quality: " + solutionQuality);
         assertTrue("Poor solution quality: " + solutionQuality, solutionQuality < 3.0e-3);
      }
   }

   @Test
   public void testRandomHandPoses() throws Exception
   {
      if (VERBOSE)
         LogTools.info("Entering: testRandomHandPoses");
      Random random = new Random(2134);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel());
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel());

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
         HumanoidKinematicsToolboxConfigurationMessage configurationMessage = new HumanoidKinematicsToolboxConfigurationMessage();
         configurationMessage.setEnableSupportPolygonConstraint(false);
         commandInputManager.submitMessage(configurationMessage);

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 150;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
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
      assertTrue("Poor worst solution quality: " + worstSolutionQuality, worstSolutionQuality < 5.0e-3);
      assertTrue("Poor average solution quality: " + averageSolutionQuality, averageSolutionQuality < 1.0e-3);
   }

   @Test
   public void testSingleSupport() throws Exception
   {
      if (VERBOSE)
         LogTools.info("Entering: testSingleSupport");
      Random random = new Random(2134);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel());
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel());
      RobotSide supportFootSide = RobotSide.LEFT;

      double averageSolutionQuality = 0.0;
      double worstSolutionQuality = -1.0;

      int numberOfTests = 30;

      RigidBodyBasics[] bodiesToControl = {randomizedFullRobotModel.getChest(), randomizedFullRobotModel.getHand(RobotSide.LEFT),
            randomizedFullRobotModel.getHand(RobotSide.RIGHT), randomizedFullRobotModel.getFoot(supportFootSide.getOppositeSide())};

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
            KinematicsToolboxCenterOfMassMessage message = MessageTools.createKinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(randomizedFullRobotModel));
            SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
            selectionMatrix.selectZAxis(false);
            message.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix));
            message.getWeights().set(MessageTools.createWeightMatrix3DMessage(1.0));
            commandInputManager.submitMessage(message);
         }

         // Disable the support polygon constraint, the randomized model isn't constrained.
         HumanoidKinematicsToolboxConfigurationMessage configurationMessage = new HumanoidKinematicsToolboxConfigurationMessage();
         configurationMessage.setEnableSupportPolygonConstraint(false);
         commandInputManager.submitMessage(configurationMessage);

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(randomizedFullRobotModel, getRobotModel(), true, false));

         int numberOfIterations = 350;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
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
      assertTrue("Poor worst solution quality: " + worstSolutionQuality, worstSolutionQuality < 3.0e-2);
      assertTrue("Poor average solution quality: " + averageSolutionQuality, averageSolutionQuality < 6.5e-3);
   }

   @Test
   public void testCenterOfMassConstraint() throws Exception
   {
      if (VERBOSE)
         LogTools.info("Entering: testCenterOfMassConstraint");

      Random random = new Random(21651);

      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel());
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
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(initialFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 250;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
         assertTrue("Poor solution quality: " + toolboxController.getSolution().getSolutionQuality(),
                    toolboxController.getSolution().getSolutionQuality() < 1.0e-4);
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
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(initialFullRobotModel, getRobotModel(), true, true));

         int numberOfIterations = 250;

         runKinematicsToolboxController(numberOfIterations);

         Point2D centerOfMass2D = new Point2D(computeCenterOfMass3D(toolboxController.getDesiredFullRobotModel()));
         assertTrue(shrunkSupportPolygon.isPointInside(centerOfMass2D, 1.0e-7), "Error: " + shrunkSupportPolygon.signedDistance(centerOfMass2D));
      }
   }

   private static KinematicsToolboxRigidBodyMessage shiftBodyMessage(RigidBodyBasics body, Tuple3DReadOnly shift, double weight)
   {
      return shiftBodyMessage(body, shift, weight, true, true);
   }

   private static KinematicsToolboxRigidBodyMessage shiftBodyMessage(RigidBodyBasics body, Tuple3DReadOnly shift, double weight, boolean controlAngular,
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
      KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage();
      message.getDesiredPositionInWorld().set(computeCenterOfMass3D(robot));
      message.getDesiredPositionInWorld().add(shift);
      message.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, false));
      message.getWeights().set(MessageTools.createWeightMatrix3DMessage(weight));
      return message;
   }

   private void runKinematicsToolboxController(int numberOfIterations) throws SimulationExceededMaximumTimeException
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

   public static FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration(DRCRobotModel robotModel)
   {
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, robotModel.getJointMap());
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null);
      SensorDataContext sensorDataContext = new SensorDataContext();
      long timestamp = drcPerfectSensorReaderFactory.getSensorReader().read(sensorDataContext);
      drcPerfectSensorReaderFactory.getSensorReader().compute(timestamp, sensorDataContext);
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

      Object<Point3D> leftFootSupportPolygon2d = capturabilityBasedStatus.getLeftFootSupportPolygon2d();
      Object<Point3D> rightFootSupportPolygon2d = capturabilityBasedStatus.getRightFootSupportPolygon2d();
      if (isLeftFootInSupport)
         contactableFeet.get(RobotSide.LEFT).getContactPointsCopy().stream().peek(cp -> cp.changeFrame(worldFrame))
                        .forEach(cp -> leftFootSupportPolygon2d.add().set(cp.getX(), cp.getY(), 0.0));
      if (isRightFootInSupport)
         contactableFeet.get(RobotSide.RIGHT).getContactPointsCopy().stream().peek(cp -> cp.changeFrame(worldFrame))
                        .forEach(cp -> rightFootSupportPolygon2d.add().set(cp.getX(), cp.getY(), 0.0));
      return capturabilityBasedStatus;
   }
}
