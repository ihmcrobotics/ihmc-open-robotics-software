package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class KinematicsToolboxControllerTest
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
   private KinematicsToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;
   private YoDouble finalSolutionQuality;

   private SimulationConstructionSet scs;
   private BlockingSimulationRunner blockingSimulationRunner;

   private Robot robot;
   private Robot ghost;
   private RobotController toolboxUpdater;

   @BeforeEach
   public void setup()
   {
      mainRegistry = new YoVariableRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      RobotDescription robotDescription = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      Pair<FloatingJointBasics, OneDoFJointBasics[]> desiredFullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDescription);
      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(MultiBodySystemTools.getRootBody(desiredFullRobotModel.getRight()[0].getSuccessor())));

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());

      double updateDT = 1.0e-3;
      toolboxController = new KinematicsToolboxController(commandInputManager,
                                                          statusOutputManager,
                                                          desiredFullRobotModel.getLeft(),
                                                          desiredFullRobotModel.getRight(),
                                                          updateDT,
                                                          yoGraphicsListRegistry,
                                                          mainRegistry);

      robot = new RobotFromDescription(robotDescription);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      RobotDescription ghostRobotDescription = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      ghostRobotDescription.setName("Ghost");
      recursivelyModifyGraphics(ghostRobotDescription.getChildrenJoints().get(0), ghostApperance);
      ghost = new RobotFromDescription(ghostRobotDescription);
      ghost.setDynamic(false);
      ghost.setGravity(0);

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

   private void snapGhostToFullRobotModel(Pair<FloatingJointBasics, OneDoFJointBasics[]> fullHumanoidRobotModel)
   {
      new JointAnglesWriter(ghost, fullHumanoidRobotModel.getLeft(), fullHumanoidRobotModel.getRight()).updateRobotConfigurationBasedOnFullRobotModel();
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
   public void testHoldBodyPose() throws Exception
   {
      Pair<FloatingJointBasics, OneDoFJointBasics[]> initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      RigidBodyBasics hand = ScrewTools.findRigidBodiesWithNames(MultiBodySystemTools.collectSubtreeSuccessors(initialFullRobotModel.getRight()),
                                                                 "handLink")[0];
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(hand));

      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      int numberOfIterations = 250;

      runKinematicsToolboxController(numberOfIterations);

      assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
      assertTrue(toolboxController.getSolution().getSolutionQuality() < 1.0e-4,
                 "Poor solution quality: " + toolboxController.getSolution().getSolutionQuality());
   }

   @Test
   public void testRandomHandPositions() throws Exception
   {
      if (VERBOSE)
         PrintTools.info(this, "Entering: testRandomHandPositions");
      Random random = new Random(2135);
      Pair<FloatingJointBasics, OneDoFJointBasics[]> initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      Pair<FloatingJointBasics, OneDoFJointBasics[]> randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration();

      for (int i = 0; i < 10; i++)
      {
         randomizeJointPositions(random, randomizedFullRobotModel, 0.6);
         RigidBodyBasics hand = ScrewTools.findRigidBodiesWithNames(MultiBodySystemTools.collectSubtreeSuccessors(randomizedFullRobotModel.getRight()),
                                                                    "handLink")[0];
         FramePoint3D desiredPosition = new FramePoint3D(hand.getBodyFixedFrame());
         desiredPosition.changeFrame(worldFrame);
         KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, desiredPosition);
         message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         commandInputManager.submitMessage(message);

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);

         int numberOfIterations = 100;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            PrintTools.info(this, "Solution quality: " + solutionQuality);
         assertTrue(solutionQuality < 1.0e-3, "Poor solution quality: " + solutionQuality);
      }
   }

   @Test
   public void testRandomHandPoses() throws Exception
   {
      if (VERBOSE)
         PrintTools.info(this, "Entering: testRandomHandPoses");
      Random random = new Random(2134);
      Pair<FloatingJointBasics, OneDoFJointBasics[]> initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      Pair<FloatingJointBasics, OneDoFJointBasics[]> randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration();

      double averageSolutionQuality = 0.0;
      double worstSolutionQuality = -1.0;

      int numberOfTests = 30;

      for (int i = 0; i < numberOfTests; i++)
      {
         randomizeJointPositions(random, randomizedFullRobotModel, 0.3);
         RigidBodyBasics hand = ScrewTools.findRigidBodiesWithNames(MultiBodySystemTools.collectSubtreeSuccessors(randomizedFullRobotModel.getRight()),
                                                                    "handLink")[0];
         KinematicsToolboxRigidBodyMessage message = holdRigidBodyCurrentPose(hand);
         message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         commandInputManager.submitMessage(message);

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);

         int numberOfIterations = 150;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
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
      assertTrue(worstSolutionQuality < 5.0e-4, "Poor worst solution quality: " + worstSolutionQuality);
      assertTrue(averageSolutionQuality < 5.0e-5, "Poor average solution quality: " + averageSolutionQuality);
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

   private Pair<FloatingJointBasics, OneDoFJointBasics[]> createFullRobotModelAtInitialConfiguration()
   {
      RobotDescription robotDescription = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      Pair<FloatingJointBasics, OneDoFJointBasics[]> fullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDescription);
      for (OneDoFJointBasics joint : fullRobotModel.getRight())
      {
         if (Double.isFinite(joint.getJointLimitLower()) && Double.isFinite(joint.getJointLimitUpper()))
            joint.setQ(0.5 * (joint.getJointLimitLower() + joint.getJointLimitUpper()));
      }
      MultiBodySystemTools.getRootBody(fullRobotModel.getRight()[0].getPredecessor()).updateFramesRecursively();
      return fullRobotModel;
   }

   private void randomizeJointPositions(Random random, Pair<FloatingJointBasics, OneDoFJointBasics[]> randomizedFullRobotModel,
                                        double percentOfMotionRangeAllowed)
   {
      for (OneDoFJointBasics joint : randomizedFullRobotModel.getRight())
      {
         double q = nextJointConfiguration(random, percentOfMotionRangeAllowed, joint);
         joint.setQ(q);
      }
      MultiBodySystemTools.getRootBody(randomizedFullRobotModel.getRight()[0].getPredecessor()).updateFramesRecursively();
   }

   public static double nextJointConfiguration(Random random, double percentOfMotionRangeAllowed, OneDoFJointReadOnly joint)
   {
      double jointLimitLower = joint.getJointLimitLower();
      if (Double.isInfinite(jointLimitLower))
         jointLimitLower = -Math.PI;
      double jointLimitUpper = joint.getJointLimitUpper();
      if (Double.isInfinite(jointLimitUpper))
         jointLimitUpper = -Math.PI;
      double rangeReduction = (1.0 - percentOfMotionRangeAllowed) * (jointLimitUpper - jointLimitLower);
      jointLimitLower += 0.5 * rangeReduction;
      jointLimitUpper -= 0.5 * rangeReduction;
      return RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper);
   }

   private RobotController createToolboxUpdater()
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot,
                                                                                   toolboxController.getDesiredRootJoint(),
                                                                                   toolboxController.getDesiredOneDoFJoint());

         @Override
         public void doControl()
         {
            if (!initializationSucceeded.getBooleanValue())
               initializationSucceeded.set(toolboxController.initialize());

            if (initializationSucceeded.getBooleanValue())
            {
               toolboxController.updateInternal();
               if (VERBOSE)
                  PrintTools.info("Solution quality: " + toolboxController.getSolution().getSolutionQuality());
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
         ArrayList<Graphics3DPrimitiveInstruction> graphics3dInstructions = linkGraphics.getGraphics3DInstructions();

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

   private RobotConfigurationData extractRobotConfigurationData(Pair<FloatingJointBasics, OneDoFJointBasics[]> initialFullRobotModel)
   {
      OneDoFJointBasics[] joints = initialFullRobotModel.getRight();
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints, new ForceSensorDefinition[0], new IMUDefinition[0]);
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));

      FloatingJointBasics rootJoint = initialFullRobotModel.getLeft();
      if (rootJoint != null)
      {
         robotConfigurationData.getRootTranslation().set(rootJoint.getJointPose().getPosition());
         robotConfigurationData.getRootOrientation().set(rootJoint.getJointPose().getOrientation());
      }
      return robotConfigurationData;
   }
}
