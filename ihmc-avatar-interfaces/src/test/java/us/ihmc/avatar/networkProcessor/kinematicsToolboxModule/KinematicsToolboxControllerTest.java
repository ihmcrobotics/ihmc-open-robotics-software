package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTestRobotsSCS2.KinematicsToolboxTestRobot;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTestRobotsSCS2.SevenDoFArm;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTestRobotsSCS2.UpperBodyWithTwoManipulators;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollisionResult;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public final class KinematicsToolboxControllerTest
{
   private static final boolean VERBOSE = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final MaterialDefinition ghostAppearance = new MaterialDefinition(ColorDefinitions.Yellow().derive(0.0, 1.0, 1.0, 0.75));
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   private CommandInputManager commandInputManager;
   private YoRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private KinematicsToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;
   private YoDouble finalSolutionQuality;

   private SimulationConstructionSet2 scs;

   private Robot robot;
   private Robot ghost;
   private Controller toolboxUpdater;

   private KinematicsToolboxTestRobot desiredFullRobotModel;

   public void setup(RobotDefinition robotDefinition, RobotDefinition ghostRobotDefinition)
   {
      mainRegistry = new YoRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      desiredFullRobotModel = KinematicsToolboxControllerTestRobotsSCS2.createInverseDynamicsRobot(robotDefinition);
      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel.getRootBody()));

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());

      double updateDT = 1.0e-3;
      toolboxController = new KinematicsToolboxController(commandInputManager,
                                                          statusOutputManager,
                                                          desiredFullRobotModel.getRootJoint(),
                                                          desiredFullRobotModel.getOneDoFJoints(),
                                                          null,
                                                          updateDT,
                                                          yoGraphicsListRegistry,
                                                          mainRegistry);

      robotDefinition.ignoreAllJoints();
      robot = new Robot(robotDefinition, SimulationConstructionSet2.inertialFrame);
      toolboxUpdater = createToolboxUpdater();
      robot.getControllerManager().addController(toolboxUpdater);

      if (ghostRobotDefinition != null)
      {
         ghostRobotDefinition.ignoreAllJoints();
         ghostRobotDefinition.setName("Ghost");
         ghostRobotDefinition.getAllRigidBodies().forEach(body -> body.getVisualDefinitions().forEach(v -> v.setMaterialDefinition(ghostAppearance)));
         ghost = new Robot(ghostRobotDefinition, SimulationConstructionSet2.inertialFrame);
      }

      if (visualize)
      {
         scs = new SimulationConstructionSet2();
         if (ghost != null)
            scs.addRobot(ghost);
         scs.addRobot(robot);
         scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));

         scs.start(true, true, true);
         scs.setCameraFocusPosition(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
      }
   }

   private void snapGhostToFullRobotModel(KinematicsToolboxTestRobot initialFullRobotModel)
   {
      MultiBodySystemTools.copyJointsState(initialFullRobotModel.getAllJoints(), ghost.getAllJoints(), JointStateType.CONFIGURATION);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void tearDown()
   {
      if (visualize)
      {
         scs.pause();
         scs.waitUntilVisualizerDown();
         LogTools.info("GUI's down");
      }

      if (mainRegistry != null)
      {
         mainRegistry.clear();
         mainRegistry = null;
      }

      if (scs != null)
      {
         scs.shutdownSession();
         scs = null;
      }

      commandInputManager = null;
      yoGraphicsListRegistry = null;
      toolboxController = null;
      initializationSucceeded = null;
      numberOfIterations = null;
      finalSolutionQuality = null;

      robot = null;
      ghost = null;
      toolboxUpdater = null;

      desiredFullRobotModel = null;

      System.gc();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testHoldBodyPose() throws Exception
   {
      SevenDoFArm robotDescription = new SevenDoFArm();
      setup(robotDescription, new SevenDoFArm());

      KinematicsToolboxTestRobot initialFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);
      snapGhostToFullRobotModel(initialFullRobotModel);

      RigidBodyBasics hand = ScrewTools.findRigidBodiesWithNames(MultiBodySystemTools.collectSubtreeSuccessors(initialFullRobotModel.getOneDoFJoints()),
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
      SevenDoFArm robotDescription = new SevenDoFArm();
      setup(robotDescription, new SevenDoFArm());

      if (VERBOSE)
         LogTools.info("Entering: testRandomHandPositions");
      Random random = new Random(2135);
      KinematicsToolboxTestRobot initialFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      KinematicsToolboxTestRobot randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);

      for (int i = 0; i < 10; i++)
      {
         randomizeJointPositions(random, randomizedFullRobotModel, 0.6);
         RigidBodyBasics hand = ScrewTools.findRigidBodiesWithNames(MultiBodySystemTools.collectSubtreeSuccessors(randomizedFullRobotModel.getOneDoFJoints()),
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
            LogTools.info("Solution quality: " + solutionQuality);
         assertTrue(solutionQuality < 1.0e-3, "Poor solution quality: " + solutionQuality);
      }
   }

   @Test
   public void testRandomJointPositionWithInputcollection() throws Exception
   {
      SevenDoFArm robotDescription = new SevenDoFArm();
      setup(robotDescription, new SevenDoFArm());

      if (VERBOSE)
         LogTools.info("Entering: testRandomJointPositionWithInputcollection");
      Random random = new Random(2135);
      KinematicsToolboxTestRobot randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);
      Map<Integer, OneDoFJointBasics> hashCodeToSolverJointMap = Stream.of(toolboxController.getDesiredOneDoFJoints())
                                                                       .collect(Collectors.toMap(JointReadOnly::hashCode, Function.identity()));

      for (int i = 0; i < 10; i++)
      {
         RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(randomizedFullRobotModel);
         int jointIndex = random.nextInt(randomizedFullRobotModel.getOneDoFJoints().length);
         OneDoFJointBasics joint = randomizedFullRobotModel.getOneDoFJoints()[jointIndex];
         joint.setQ(EuclidCoreRandomTools.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper()));

         int jointHashCode = joint.hashCode();
         double desiredPosition = joint.getQ();
         KinematicsToolboxInputCollectionMessage inputCollectionMessage = new KinematicsToolboxInputCollectionMessage();
         KinematicsToolboxOneDoFJointMessage jointMessage = new KinematicsToolboxOneDoFJointMessage();
         jointMessage.setJointHashCode(jointHashCode);
         jointMessage.setWeight(1.0);
         jointMessage.setDesiredPosition(desiredPosition);
         inputCollectionMessage.getJointInputs().add().set(jointMessage);

         commandInputManager.submitMessage(inputCollectionMessage);

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);

         int numberOfIterations = 100;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            LogTools.info("Solution quality: " + solutionQuality);
         assertTrue(solutionQuality < 1.0e-3, "Poor solution quality: " + solutionQuality);

         OneDoFJointBasics solverJoint = hashCodeToSolverJointMap.get(joint.hashCode());
         assertEquals(joint.getQ(), solverJoint.getQ(), 1.0e-7, "Error too large for: " + joint.getName());
      }
   }

   @Test
   public void testRandomHandPoses() throws Exception
   {
      SevenDoFArm robotDescription = new SevenDoFArm();
      setup(robotDescription, new SevenDoFArm());

      if (VERBOSE)
         LogTools.info("Entering: testRandomHandPoses");
      Random random = new Random(2134);
      KinematicsToolboxTestRobot initialFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      KinematicsToolboxTestRobot randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);

      double averageSolutionQuality = 0.0;
      double worstSolutionQuality = -1.0;

      int numberOfTests = 30;

      for (int i = 0; i < numberOfTests; i++)
      {
         randomizeJointPositions(random, randomizedFullRobotModel, 0.3);
         RigidBodyBasics hand = ScrewTools.findRigidBodiesWithNames(MultiBodySystemTools.collectSubtreeSuccessors(randomizedFullRobotModel.getOneDoFJoints()),
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
            LogTools.info("Solution quality: " + solutionQuality);
         averageSolutionQuality += solutionQuality / numberOfTests;
         worstSolutionQuality = Math.max(worstSolutionQuality, solutionQuality);
      }

      if (VERBOSE)
      {
         LogTools.info("Solution quality: average = " + averageSolutionQuality + ", worst = " + worstSolutionQuality);
      }
      assertTrue(worstSolutionQuality < 5.0e-4, "Poor worst solution quality: " + worstSolutionQuality);
      assertTrue(averageSolutionQuality < 5.0e-5, "Poor average solution quality: " + averageSolutionQuality);
   }

   @Test
   public void testRandomDualHandPositions() throws Exception
   {
      UpperBodyWithTwoManipulators robotDescription = new UpperBodyWithTwoManipulators();
      setup(robotDescription, new UpperBodyWithTwoManipulators());

      if (VERBOSE)
         LogTools.info("Entering: testRandomDualHandPositions");
      Random random = new Random(2135);
      KinematicsToolboxTestRobot initialFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      KinematicsToolboxTestRobot randomizedFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);

      for (int i = 0; i < 10; i++)
      {
         randomizeJointPositions(random, randomizedFullRobotModel, 0.6);

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = ScrewTools.findRigidBodiesWithNames(MultiBodySystemTools.collectSubtreeSuccessors(randomizedFullRobotModel.getOneDoFJoints()),
                                                                       robotSide.getCamelCaseName() + "HandLink")[0];
            FramePoint3D desiredPosition = new FramePoint3D(hand.getBodyFixedFrame());
            desiredPosition.changeFrame(worldFrame);
            KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, desiredPosition);
            message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
            commandInputManager.submitMessage(message);
         }

         snapGhostToFullRobotModel(randomizedFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);

         int numberOfIterations = 100;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            LogTools.info("Solution quality: " + solutionQuality);
         assertTrue(solutionQuality < 1.0e-3, "Poor solution quality: " + solutionQuality);
      }
   }

   @Test
   public void testRandomDualHandPositionsCollisionWithTorso() throws Exception
   {
      UpperBodyWithTwoManipulators robotDescription = new UpperBodyWithTwoManipulators();

      Capsule3D torsoCollisionShape = new Capsule3D(0.2, 0.2);
      torsoCollisionShape.getPosition().addZ(0.2);
      Sphere3D handCollisionShape = new Sphere3D(0.1);
      handCollisionShape.getPosition().addZ(0.05);

      RigidBodyDefinition torsoLinkDescription = robotDescription.getRigidBodyDefinition("torsoLink");
      SideDependentList<RigidBodyDefinition> handLinkDescriptions = new SideDependentList<>(side -> robotDescription.getRigidBodyDefinition(side.getCamelCaseName()
            + "HandLink"));

      ColorDefinition collisionGraphicAppearance = ColorDefinitions.SpringGreen().derive(0, 1.0, 0.5, 0.15);

      VisualDefinitionFactory torsoCollisionGraphic = new VisualDefinitionFactory();
      torsoCollisionGraphic.appendTranslation(torsoCollisionShape.getPosition());
      torsoCollisionGraphic.addCapsule(torsoCollisionShape.getLength() + 2.0 * torsoCollisionShape.getRadius(),
                                       torsoCollisionShape.getRadius(), // the 2nd term is removed internally.
                                       collisionGraphicAppearance);
      torsoLinkDescription.getVisualDefinitions().addAll(torsoCollisionGraphic.getVisualDefinitions());
      VisualDefinitionFactory handCollisionGraphic = new VisualDefinitionFactory();
      handCollisionGraphic.appendTranslation(handCollisionShape.getPosition());
      handCollisionGraphic.addSphere(handCollisionShape.getRadius(), collisionGraphicAppearance);
      handLinkDescriptions.values().forEach(linkDescription -> linkDescription.getVisualDefinitions().addAll(handCollisionGraphic.getVisualDefinitions()));

      setup(robotDescription, null);

      RigidBodyBasics rootBody = desiredFullRobotModel.getRootBody();
      List<? extends RigidBodyBasics> rigidBodies = rootBody.subtreeList();

      RigidBodyBasics torso = rigidBodies.stream().filter(body -> body.getName().equals("torsoLink")).findFirst().get();
      SideDependentList<RigidBodyBasics> hands = new SideDependentList<>(side -> rigidBodies.stream()
                                                                                            .filter(body -> body.getName()
                                                                                                                .equals(side.getCamelCaseName() + "HandLink"))
                                                                                            .findFirst().get());

      Collidable torsoCollidable = new Collidable(torso, 0b001, 0b110, new FrameCapsule3D(torso.getParentJoint().getFrameAfterJoint(), torsoCollisionShape));

      SideDependentList<Collidable> handCollidables = new SideDependentList<>(side ->
      {
         RigidBodyBasics hand = hands.get(side);
         int collisionMask = side == RobotSide.LEFT ? 0b010 : 0b100;
         int collisionGroup = 0b001;
         ReferenceFrame shapeFrame = hand.getParentJoint().getFrameAfterJoint();
         return new Collidable(hand, collisionMask, collisionGroup, new FrameSphere3D(shapeFrame, handCollisionShape));
      });

      toolboxController.registerRobotCollidable(torsoCollidable);
      toolboxController.registerRobotCollidables(handCollidables);

      if (VERBOSE)
         LogTools.info("Entering: testRandomDualHandPositionsCollisionWithTorso");
      Random random = new Random(2135);
      KinematicsToolboxTestRobot initialFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDescription);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      for (int i = 0; i < 10; i++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint3D desiredPosition = new FramePoint3D(worldFrame);
            desiredPosition.setZ(EuclidCoreTools.interpolate(torsoCollisionShape.getBottomCenter().getZ(),
                                                             torsoCollisionShape.getTopCenter().getZ(),
                                                             random.nextDouble()));
            double distanceFromCenter = random.nextDouble() * (torsoCollisionShape.getRadius() + handCollisionShape.getRadius());
            desiredPosition.add(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, distanceFromCenter));

            KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hands.get(robotSide), desiredPosition);
            message.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, false));
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
            commandInputManager.submitMessage(message);
         }

         toolboxController.updateRobotConfigurationData(robotConfigurationData);

         int numberOfIterations = 250;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            LogTools.info("Solution quality: " + solutionQuality);

         for (RobotSide robotSide : RobotSide.values)
         {
            CollisionResult collision = torsoCollidable.evaluateCollision(handCollidables.get(robotSide));
            assertTrue(collision.getCollisionData().getSignedDistance() > -1.0e-3);
         }
      }
   }

   @Test
   public void testRandomHandPositionsCollisionWithStatic() throws Exception
   {
      UpperBodyWithTwoManipulators robotDefinition = new UpperBodyWithTwoManipulators();

      Capsule3D torsoCollisionShape = new Capsule3D(0.2, 0.2);
      torsoCollisionShape.getPosition().addZ(0.2);
      Sphere3D handCollisionShape = new Sphere3D(0.1);
      handCollisionShape.getPosition().addZ(0.05);

      SideDependentList<RigidBodyDefinition> handLinkDescriptions = new SideDependentList<>(side -> robotDefinition.getRigidBodyDefinition(side.getCamelCaseName()
            + "HandLink"));

      ColorDefinition collisionGraphicAppearance = ColorDefinitions.SpringGreen().derive(0, 1.0, 0.5, 0.15);

      VisualDefinitionFactory handCollisionGraphic = new VisualDefinitionFactory();
      handCollisionGraphic.appendTranslation(handCollisionShape.getPosition());
      handCollisionGraphic.addSphere(handCollisionShape.getRadius(), collisionGraphicAppearance);
      handLinkDescriptions.values().forEach(linkDescription -> linkDescription.getVisualDefinitions().addAll(handCollisionGraphic.getVisualDefinitions()));

      setup(robotDefinition, null);

      RigidBodyBasics rootBody = desiredFullRobotModel.getRootBody();
      List<? extends RigidBodyBasics> rigidBodies = rootBody.subtreeList();

      SideDependentList<RigidBodyBasics> hands = new SideDependentList<>(side -> rigidBodies.stream()
                                                                                            .filter(body -> body.getName()
                                                                                                                .equals(side.getCamelCaseName() + "HandLink"))
                                                                                            .findFirst().get());

      SideDependentList<Collidable> handCollidables = new SideDependentList<>(side ->
      {
         RigidBodyBasics hand = hands.get(side);
         int collisionMask = side == RobotSide.LEFT ? 0b010 : 0b100;
         int collisionGroup = 0b001;
         ReferenceFrame shapeFrame = hand.getParentJoint().getFrameAfterJoint();
         return new Collidable(hand, collisionMask, collisionGroup, new FrameSphere3D(shapeFrame, handCollisionShape));
      });

      toolboxController.registerRobotCollidables(handCollidables);

      Sphere3D sphere = new Sphere3D(0.0, 0.75, 0.20, 0.5);

      if (visualize)
         scs.addStaticVisual(new VisualDefinition(sphere.getCentroid(),
                                                  new Sphere3DDefinition(sphere.getRadius()),
                                                  new MaterialDefinition(ColorDefinitions.DarkSalmon())));

      FrameSphere3D staticFrameSphere = new FrameSphere3D(worldFrame, sphere);
      Collidable staticCollidable = new Collidable(null, 0b001, 0b110, staticFrameSphere);
      toolboxController.registerStaticCollidable(staticCollidable);

      if (VERBOSE)
         LogTools.info("Entering: testRandomDualHandPositionsCollisionWithTorso");
      Random random = new Random(2135);
      KinematicsToolboxTestRobot initialFullRobotModel = createFullRobotModelAtInitialConfiguration(robotDefinition);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      for (int i = 0; i < 10; i++)
      {
         RobotSide robotSide = RobotSide.LEFT;

         FramePoint3D desiredPosition = new FramePoint3D(worldFrame);
         desiredPosition.setY(EuclidCoreRandomTools.nextDouble(random, -0.10, 0.10));
         desiredPosition.setY(EuclidCoreRandomTools.nextDouble(random, +0.15, 0.60));
         desiredPosition.setZ(EuclidCoreRandomTools.nextDouble(random, +0.00, 0.50));

         KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hands.get(robotSide), desiredPosition);
         message.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, false));
         message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
         commandInputManager.submitMessage(message);

         toolboxController.updateRobotConfigurationData(robotConfigurationData);

         int numberOfIterations = 250;

         runKinematicsToolboxController(numberOfIterations);

         assertTrue(initializationSucceeded.getBooleanValue(), KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            LogTools.info("Solution quality: " + solutionQuality);

         CollisionResult collision = staticCollidable.evaluateCollision(handCollidables.get(robotSide));
         assertTrue(collision.getCollisionData().getSignedDistance() > -1.0e-3);
      }
   }

   private void runKinematicsToolboxController(int numberOfIterations)
   {
      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         for (int i = 0; i < numberOfIterations; i++)
         {
            scs.simulateNow(1);
         }
      }
      else
      {
         for (int i = 0; i < numberOfIterations; i++)
            toolboxUpdater.doControl();
      }

      finalSolutionQuality.set(toolboxController.getSolution().getSolutionQuality());
   }

   private KinematicsToolboxTestRobot createFullRobotModelAtInitialConfiguration(RobotDefinition robotDefinition)
   {
      KinematicsToolboxTestRobot fullRobotModel = KinematicsToolboxControllerTestRobotsSCS2.createInverseDynamicsRobot(robotDefinition);

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         if (Double.isFinite(joint.getJointLimitLower()) && Double.isFinite(joint.getJointLimitUpper()))
            joint.setQ(0.5 * (joint.getJointLimitLower() + joint.getJointLimitUpper()));
      }
      fullRobotModel.getRootBody().updateFramesRecursively();
      return fullRobotModel;
   }

   private void randomizeJointPositions(Random random, KinematicsToolboxTestRobot randomizedFullRobotModel, double percentOfMotionRangeAllowed)
   {
      for (OneDoFJointBasics joint : randomizedFullRobotModel.getOneDoFJoints())
      {
         double q = nextJointConfiguration(random, percentOfMotionRangeAllowed, joint);
         joint.setQ(q);
      }
      randomizedFullRobotModel.getRootBody().updateFramesRecursively();
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

   private Controller createToolboxUpdater()
   {
      return new Controller()
      {
         @Override
         public void doControl()
         {
            if (!initializationSucceeded.getBooleanValue())
               initializationSucceeded.set(toolboxController.initialize());

            if (initializationSucceeded.getBooleanValue())
            {
               toolboxController.updateInternal();
               if (VERBOSE)
                  LogTools.info("Solution quality: " + toolboxController.getSolution().getSolutionQuality());
               // TODO Not sure if the joints are in the same order
               MultiBodySystemTools.copyJointsState(desiredFullRobotModel.getAllJoints(), robot.getAllJoints(), JointStateType.CONFIGURATION);
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

   private RobotConfigurationData extractRobotConfigurationData(KinematicsToolboxTestRobot initialFullRobotModel)
   {
      OneDoFJointBasics[] joints = initialFullRobotModel.getOneDoFJoints();
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints, new ForceSensorDefinition[0], new IMUDefinition[0]);
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));

      FloatingJointBasics rootJoint = initialFullRobotModel.getRootJoint();
      if (rootJoint != null)
      {
         robotConfigurationData.getRootPosition().set(rootJoint.getJointPose().getPosition());
         robotConfigurationData.getRootOrientation().set(rootJoint.getJointPose().getOrientation());
      }
      return robotConfigurationData;
   }
}
