package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static org.junit.Assert.assertTrue;
import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.Pair;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class KinematicsToolboxCollisionDetectionTest
{
   private static final boolean VERBOSE = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
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

   @Before
   public void setup()
   {
      mainRegistry = new YoVariableRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      RobotDescription robotDescription = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> desiredFullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDescription);
      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(ScrewTools.getRootBody(desiredFullRobotModel.getRight()[0].getSuccessor())));

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());

      toolboxController = new KinematicsToolboxController(commandInputManager, statusOutputManager, desiredFullRobotModel.getLeft(),
                                                          desiredFullRobotModel.getRight(), yoGraphicsListRegistry, mainRegistry);

      robot = new RobotFromDescription(robotDescription);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      RobotDescription ghostRobotDescription = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      ghostRobotDescription.setName("Ghost");
      recursivelyModyfyGraphics(ghostRobotDescription.getChildrenJoints().get(0));
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

   private void snapGhostToFullRobotModel(Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> fullHumanoidRobotModel)
   {
      new JointAnglesWriter(ghost, fullHumanoidRobotModel.getLeft(), fullHumanoidRobotModel.getRight()).updateRobotConfigurationBasedOnFullRobotModel();
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

   @Test
   public void testHoldBodyPose() throws Exception
   {
      Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> initialFullRobotModel = createFullRobotModelAtInitialConfiguration();
      snapGhostToFullRobotModel(initialFullRobotModel);

      RigidBody hand = ScrewTools.findRigidBodiesWithNames(ScrewTools.computeRigidBodiesAfterThisJoint(initialFullRobotModel.getRight()), "handLink")[0];
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(hand));

      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      int numberOfIterations = 250;

      runKinematicsToolboxController(numberOfIterations);

      assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
      assertTrue("Poor solution quality: " + toolboxController.getSolution().getSolutionQuality(),
                 toolboxController.getSolution().getSolutionQuality() < 1.0e-4);
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

   private Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> createFullRobotModelAtInitialConfiguration()
   {
      RobotDescription robotDescription = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> fullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDescription);
      for (OneDoFJoint joint : fullRobotModel.getRight())
      {
         if (Double.isFinite(joint.getJointLimitLower()) && Double.isFinite(joint.getJointLimitUpper()))
            joint.setQ(0.5 * (joint.getJointLimitLower() + joint.getJointLimitUpper()));
      }
      return fullRobotModel;
   }

   private RobotController createToolboxUpdater()
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot, toolboxController.getDesiredRootJoint(),
                                                                                   toolboxController.getDesiredOneDoFJoint());

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
         recursivelyModyfyGraphics(child);
      }
   }

   private RobotConfigurationData extractRobotConfigurationData(Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> initialFullRobotModel)
   {
      OneDoFJoint[] joints = initialFullRobotModel.getRight();
      RobotConfigurationData robotConfigurationData = new RobotConfigurationData(joints, new ForceSensorDefinition[0], null, new IMUDefinition[0]);
      robotConfigurationData.setJointState(Arrays.stream(joints).collect(Collectors.toList()));

      FloatingInverseDynamicsJoint rootJoint = initialFullRobotModel.getLeft();
      if (rootJoint != null)
      {
         robotConfigurationData.setRootTranslation(rootJoint.getTranslationForReading());
         robotConfigurationData.setRootOrientation(rootJoint.getRotationForReading());
      }
      return robotConfigurationData;
   }
}
