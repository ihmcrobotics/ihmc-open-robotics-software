package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static org.junit.Assert.assertTrue;
import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.KinematicsToolboxCenterOfMassMessage;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class AvatarHumanoidKinematicsToolboxControllerTest implements MultiRobotTestInterface
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

   @Before
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

      toolboxController = new HumanoidKinematicsToolboxController(commandInputManager, statusOutputManager, desiredFullRobotModel, yoGraphicsListRegistry,
                                                          mainRegistry);

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName("Ghost");
      recursivelyModyfyGraphics(robotDescription.getChildrenJoints().get(0));
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

   @After
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

   @Test(timeout = 30000)
   public void testHoldBodyPose() throws Exception
   {
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getPelvis()));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getChest()));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getHand(RobotSide.LEFT)));
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(initialFullRobotModel.getHand(RobotSide.RIGHT)));

      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(true, true));

      int numberOfIterations = 250;

      runKinematicsToolboxController(numberOfIterations);

      assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
      assertTrue("Poor solution quality: " + toolboxController.getSolution().getSolutionQuality(),
                 toolboxController.getSolution().getSolutionQuality() < 1.0e-4);
   }

   @Test(timeout = 30000)
   public void testRandomHandPositions() throws Exception
   {
      if (VERBOSE)
         PrintTools.info(this, "Entering: testRandomHandPositions");
      Random random = new Random(2135);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration();

      for (int i = 0; i < 10; i++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            randomizeArmJointPositions(random, robotSide, randomizedFullRobotModel, 0.6);
            RigidBody hand = randomizedFullRobotModel.getHand(robotSide);
            FramePoint3D desiredPosition = new FramePoint3D(hand.getBodyFixedFrame());
            desiredPosition.changeFrame(worldFrame);
            KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(hand, desiredPosition);
            message.setWeight(20.0);
            commandInputManager.submitMessage(message);
         }

         { // Setup CoM message
            KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(randomizedFullRobotModel));
            SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
            selectionMatrix.selectZAxis(false);
            message.setSelectionMatrix(selectionMatrix);
            message.setWeight(1.0);
            commandInputManager.submitMessage(message);
         }

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(true, true));

         int numberOfIterations = 100;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            PrintTools.info(this, "Solution quality: " + solutionQuality);
         assertTrue("Poor solution quality: " + solutionQuality, solutionQuality < 1.0e-4);
      }
   }

   @Test(timeout = 30000)
   public void testRandomHandPoses() throws Exception
   {
      if (VERBOSE)
         PrintTools.info(this, "Entering: testRandomHandPoses");
      Random random = new Random(2134);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration();

      double averageSolutionQuality = 0.0;
      double worstSolutionQuality = -1.0;

      int numberOfTests = 30;

      for (int i = 0; i < numberOfTests; i++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            randomizeArmJointPositions(random, robotSide, randomizedFullRobotModel, 0.4);
            KinematicsToolboxRigidBodyMessage message = holdRigidBodyCurrentPose(randomizedFullRobotModel.getHand(robotSide));
            message.setWeight(20.0);
            commandInputManager.submitMessage(message);
         }

         { // Setup CoM message
            KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(randomizedFullRobotModel));
            SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
            selectionMatrix.selectZAxis(false);
            message.setSelectionMatrix(selectionMatrix);
            message.setWeight(1.0);
            commandInputManager.submitMessage(message);
         }

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(true, true));

         int numberOfIterations = 150;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            PrintTools.info(this, "Solution quality: " + solutionQuality);
         averageSolutionQuality += solutionQuality / numberOfTests;
         worstSolutionQuality = Math.max(worstSolutionQuality, solutionQuality);
      }

      if (VERBOSE)
      {
         PrintTools.info(this, "Solution quality: average = " + averageSolutionQuality + ", worst = " + worstSolutionQuality);
      }
      assertTrue("Poor worst solution quality: " + worstSolutionQuality, worstSolutionQuality < 5.0e-4);
      assertTrue("Poor average solution quality: " + averageSolutionQuality, averageSolutionQuality < 5.0e-5);
   }

   @Test(timeout = 30000)
   public void testSingleSupport() throws Exception
   {
      if (VERBOSE)
         PrintTools.info(this, "Entering: testSingleSupport");
      Random random = new Random(2134);
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullHumanoidRobotModel randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration();
      RobotSide supportFootSide = RobotSide.LEFT;

      double averageSolutionQuality = 0.0;
      double worstSolutionQuality = -1.0;

      int numberOfTests = 30;

      RigidBody[] bodiesToControl = {
            randomizedFullRobotModel.getChest(),
            randomizedFullRobotModel.getHand(RobotSide.LEFT),
            randomizedFullRobotModel.getHand(RobotSide.RIGHT),
            randomizedFullRobotModel.getFoot(supportFootSide.getOppositeSide())};

      for (int i = 0; i < numberOfTests; i++)
      {
         randomizeJointPositions(random, randomizedFullRobotModel.getOneDoFJoints(), 0.5);
         randomizedFullRobotModel.updateFrames();
         ReferenceFrame rootJointFrame = randomizedFullRobotModel.getRootJoint().getFrameAfterJoint();
         ReferenceFrame supportFootFrame = randomizedFullRobotModel.getFoot(supportFootSide).getBodyFixedFrame();
         RigidBodyTransform transformFromRootJointToWorldFrame = rootJointFrame.getTransformToDesiredFrame(supportFootFrame);
         RigidBodyTransform initialSupportFootTransform = initialFullRobotModel.getFoot(supportFootSide).getBodyFixedFrame().getTransformToWorldFrame();
         transformFromRootJointToWorldFrame.preMultiply(initialSupportFootTransform);
         
         randomizedFullRobotModel.getRootJoint().setPositionAndRotation(transformFromRootJointToWorldFrame);
         randomizedFullRobotModel.updateFrames();

         for (RigidBody rigidBody : bodiesToControl)
         {
            FramePoint3D desiredPosition = new FramePoint3D(rigidBody.getBodyFixedFrame());
            desiredPosition.changeFrame(worldFrame);
            KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(rigidBody, desiredPosition);
            message.setWeight(20.0);
            commandInputManager.submitMessage(message);
         }

         { // Setup CoM message
            KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage(computeCenterOfMass3D(randomizedFullRobotModel));
            SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
            selectionMatrix.selectZAxis(false);
            message.setSelectionMatrix(selectionMatrix);
            message.setWeight(1.0);
            commandInputManager.submitMessage(message);
         }

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(true, false));

         int numberOfIterations = 250;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            PrintTools.info(this, "Solution quality: " + solutionQuality);
         averageSolutionQuality += solutionQuality / numberOfTests;
         worstSolutionQuality = Math.max(worstSolutionQuality, solutionQuality);
      }

      if (VERBOSE)
      {
         PrintTools.info(this, "Solution quality: average = " + averageSolutionQuality + ", worst = " + worstSolutionQuality);
      }
      assertTrue("Poor worst solution quality: " + worstSolutionQuality, worstSolutionQuality < 2.0e-3);
      assertTrue("Poor average solution quality: " + averageSolutionQuality, averageSolutionQuality < 1.0e-4);
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

   private void randomizeArmJointPositions(Random random, RobotSide robotSide, FullHumanoidRobotModel robotModelToModify)
   {
      randomizeArmJointPositions(random, robotSide, robotModelToModify, 1.0);
   }

   private void randomizeArmJointPositions(Random random, RobotSide robotSide, FullHumanoidRobotModel robotModelToModify, double percentOfMotionRangeAllowed)
   {
      RigidBody chest = robotModelToModify.getChest();
      RigidBody hand = robotModelToModify.getHand(robotSide);
      randomizeKinematicsChainPositions(random, chest, hand, percentOfMotionRangeAllowed);
   }

   private void randomizeKinematicsChainPositions(Random random, RigidBody base, RigidBody body)
   {
      randomizeKinematicsChainPositions(random, base, body, 1.0);
   }

   private void randomizeKinematicsChainPositions(Random random, RigidBody base, RigidBody body, double percentOfMotionRangeAllowed)
   {
      percentOfMotionRangeAllowed = MathTools.clamp(percentOfMotionRangeAllowed, 0.0, 1.0);

      OneDoFJoint[] joints = ScrewTools.createOneDoFJointPath(base, body);

      randomizeJointPositions(random, joints, percentOfMotionRangeAllowed);
   }

   private void randomizeJointPositions(Random random, OneDoFJoint[] joints, double percentOfMotionRangeAllowed)
   {
      for (OneDoFJoint joint : joints)
      {
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();
         double rangeReduction = (1.0 - percentOfMotionRangeAllowed) * (jointLimitUpper - jointLimitLower);
         jointLimitLower += 0.5 * rangeReduction;
         jointLimitUpper -= 0.5 * rangeReduction;
         joint.setQ(RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper));
      }
   }

   private FramePoint3D computeCenterOfMass3D(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      CenterOfMassCalculator calculator = new CenterOfMassCalculator(fullHumanoidRobotModel.getElevator(), worldFrame);
      calculator.compute();
      return calculator.getCenterOfMass();
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

   private static void recursivelyModyfyGraphics(JointDescription joint)
   {
      if (joint == null)
         return;
      LinkDescription link = joint.getLink();
      if (link == null)
         return;
      LinkGraphicsDescription linkGraphics = link.getLinkGraphics();
      if (linkGraphics == null)
         return;

      ArrayList<Graphics3DPrimitiveInstruction> graphics3dInstructions = linkGraphics.getGraphics3DInstructions();

      if (graphics3dInstructions == null)
         return;

      for (Graphics3DPrimitiveInstruction primitive : graphics3dInstructions)
      {
         if (primitive instanceof Graphics3DAddModelFileInstruction)
         {
            Graphics3DAddModelFileInstruction modelInstruction = (Graphics3DAddModelFileInstruction) primitive;
            modelInstruction.setAppearance(ghostApperance);
         }
      }

      if (joint.getChildrenJoints() == null)
         return;

      for (JointDescription child : joint.getChildrenJoints())
      {
         recursivelyModyfyGraphics(child);
      }
   }

   private RobotConfigurationData extractRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      OneDoFJoint[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      RobotConfigurationData robotConfigurationData = new RobotConfigurationData(joints, new ForceSensorDefinition[0], null, new IMUDefinition[0]);
      robotConfigurationData.setJointState(Arrays.stream(joints).collect(Collectors.toList()));
      robotConfigurationData.setRootTranslation(fullRobotModel.getRootJoint().getTranslationForReading());
      robotConfigurationData.setRootOrientation(fullRobotModel.getRootJoint().getRotationForReading());
      return robotConfigurationData;
   }

   private CapturabilityBasedStatus createCapturabilityBasedStatus(boolean isLeftFootInSupport, boolean isRightFootInSupport)
   {
      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();
      capturabilityBasedStatus.leftFootSupportPolygonLength = isLeftFootInSupport ? 1 : 0;
      capturabilityBasedStatus.rightFootSupportPolygonLength = isRightFootInSupport ? 1 : 0;
      return capturabilityBasedStatus;
   }
}
