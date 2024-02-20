package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollisionResult;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerOutputBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleFunction;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.*;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.RelativeEndEffectorControlTest.circlePositionAt;

@Tag("humanoid-toolbox")
public abstract class KinematicsStreamingToolboxControllerTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected static final double toolboxControllerPeriod = 5.0e-3;
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected static final MaterialDefinition ghostMaterial = new MaterialDefinition(ColorDefinitions.Yellow().derive(0, 1, 1, 0.25));
   protected static final boolean visualize = simulationTestingParameters.getCreateGUI();

   protected CommandInputManager commandInputManager;
   protected StatusMessageOutputManager statusOutputManager;
   protected YoRegistry toolboxRegistry;
   protected YoGraphicsListRegistry yoGraphicsListRegistry;
   protected FullHumanoidRobotModel desiredFullRobotModel;
   protected KinematicsStreamingToolboxController toolboxController;

   protected SCS2AvatarTestingSimulation simulationTestHelper;

   protected Robot robot, ghost;
   protected ROS2Node ros2Node;
   protected IHMCROS2Publisher<KinematicsStreamingToolboxInputMessage> inputPublisher;
   protected IHMCROS2Publisher<ToolboxStateMessage> statePublisher;
   protected ROS2Topic<?> controllerInputTopic;
   protected ROS2Topic<?> controllerOutputTopic;
   protected ROS2Topic<?> toolboxInputTopic;
   protected ROS2Topic<?> toolboxOutputTopic;
   protected ScheduledExecutorService executor;

   /**
    * Returns a <b>new</b> instance of the robot model that will be modified in this test to create
    * ghost robots.
    */
   public abstract DRCRobotModel newRobotModel();

   public void setupWithWalkingController(KinematicsStreamingToolboxParameters toolboxParameters, Controller... additionalGhostControllers)
   {
      DRCRobotModel robotModel = newRobotModel();
      RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();

      String robotName = robotModel.getSimpleRobotName();

      RobotDefinition ghostDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      RobotDefinitionTools.setRobotDefinitionMaterial(ghostDefinition, ghostMaterial);
      ghostDefinition.ignoreAllJoints();
      ghostDefinition.setName("ghost");
      addCollisionVisuals(robotModel, collisionModel, ghostDefinition);
      ghost = new Robot(ghostDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
      hideRobot(ghost);

      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             testEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.addSecondaryRobot(ghost);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      createToolboxController(robotModel, toolboxParameters, collisionModel);

      ros2Node = simulationTestHelper.getROS2Node();

      controllerInputTopic = ROS2Tools.getControllerInputTopic(robotName);
      controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      toolboxInputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
      toolboxOutputTopic = KinematicsStreamingToolboxModule.getOutputTopic(robotName);

      RealtimeROS2Node toolboxROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS, "toolbox_node");
      new ControllerNetworkSubscriber(toolboxInputTopic, commandInputManager, toolboxOutputTopic, statusOutputManager, toolboxROS2Node);
      IHMCROS2Publisher<WholeBodyTrajectoryMessage> outputPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                         WholeBodyTrajectoryMessage.class,
                                                                                                         controllerInputTopic);
      toolboxController.setTrajectoryMessagePublisher(outputPublisher::publish);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    controllerOutputTopic,
                                                    s -> toolboxController.updateRobotConfigurationData(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    CapturabilityBasedStatus.class,
                                                    controllerOutputTopic,
                                                    s -> toolboxController.updateCapturabilityBasedStatus(s.takeNextData()));

      inputPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, KinematicsStreamingToolboxInputMessage.class, toolboxInputTopic);
      statePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ToolboxStateMessage.class, toolboxInputTopic);

      AtomicReference<KinematicsToolboxOutputStatus> toolboxViz = new AtomicReference<>(null);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, KinematicsToolboxOutputStatus.class, toolboxOutputTopic, s -> toolboxViz.set(s.takeNextData()));

      Controller toolboxUpdater = new Controller()
      {
         private final JointReadOnly[] desiredJoints = MultiBodySystemTools.collectSubtreeJoints(desiredFullRobotModel.getElevator());
         private final ControllerOutputBasics scsInput = ghost.getControllerOutput();

         @Override
         public void doControl()
         {
            toolboxController.update();

            for (JointReadOnly joint : desiredJoints)
            {
               scsInput.getJointOutput(joint).setConfiguration(joint);
            }
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return toolboxRegistry;
         }
      };
      ghost.addThrottledController(toolboxUpdater, toolboxControllerPeriod);

      if (additionalGhostControllers != null)
      {
         for (Controller ghostController : additionalGhostControllers)
            ghost.addThrottledController(ghostController, toolboxControllerPeriod);
      }

      toolboxROS2Node.spin();
      simulationTestHelper.start();
   }

   public void setupNoWalkingController(RobotCollisionModel collisionModel)
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DRCRobotModel robotModel = newRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      RobotDefinition robotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      robotDefinition.ignoreAllJoints();
      addCollisionVisuals(robotModel, collisionModel, robotDefinition);
      robot = new Robot(robotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
      createToolboxController(robotModel, null, collisionModel);

      RobotDefinition ghostDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      RobotDefinitionTools.setRobotDefinitionMaterial(ghostDefinition, ghostMaterial);
      ghostDefinition.ignoreAllJoints();
      ghostDefinition.setName("ghost");
      ghost = new Robot(ghostDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
      hideRobot(ghost);

      if (visualize)
      {
         SimulationConstructionSet2 scs = new SimulationConstructionSet2();
         scs.addRobot(robot);
         if (ghost != null)
            scs.addRobot(ghost);
         scs.getRootRegistry().addChild(toolboxRegistry);
         scs.setDT(toolboxControllerPeriod);
         scs.initializeBufferRecordTickPeriod(1);

         simulationTestHelper = new SCS2AvatarTestingSimulation(scs, robotModel, desiredFullRobotModel, yoGraphicsListRegistry, simulationTestingParameters);
         simulationTestHelper.setKeepSCSUp(simulationTestingParameters.getKeepSCSUp());
         simulationTestHelper.start(false);
         simulationTestHelper.setCamera(new Point3D(0, 0, 1), new Point3D(6, 0, 1));
      }
   }

   private void addCollisionVisuals(FullHumanoidRobotModelFactory fullRobotModelFactory, RobotCollisionModel collisionModel, RobotDefinition robotDefinition)
   {
      if (collisionModel != null)
      {
         RigidBodyBasics elevator = fullRobotModelFactory.createFullRobotModel().getElevator();
         List<Collidable> collidables = collisionModel.getRobotCollidables(elevator);

         for (Collidable collidable : collidables)
         {
            List<VisualDefinition> visuals = getCollisionVisuals(collidable);
            robotDefinition.getRigidBodyDefinition(collidable.getRigidBody().getName()).getVisualDefinitions().addAll(visuals);
         }
      }
   }

   private void createToolboxController(DRCRobotModel robotModel, KinematicsStreamingToolboxParameters toolboxParameters, RobotCollisionModel collisionModel)
   {
      desiredFullRobotModel = robotModel.createFullRobotModel();
      toolboxRegistry = new YoRegistry("toolboxMain");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
      statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());

      if (toolboxParameters == null)
         toolboxParameters = KinematicsStreamingToolboxParameters.defaultParameters();

      toolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                   statusOutputManager,
                                                                   toolboxParameters,
                                                                   desiredFullRobotModel,
                                                                   robotModel,
                                                                   robotModel.getControllerDT(),
                                                                   toolboxControllerPeriod,
                                                                   yoGraphicsListRegistry,
                                                                   toolboxRegistry);
      toolboxController.setCollisionModel(collisionModel);
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      desiredFullRobotModel = null;
      toolboxRegistry = null;
      yoGraphicsListRegistry = null;
      commandInputManager = null;
      toolboxController = null;
      robot = null;
      ghost = null;
      ros2Node = null;
      inputPublisher = null;
      statePublisher = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testHandMotionWithCollision()
   {
      DRCRobotModel robotModel = newRobotModel();
      setupNoWalkingController(robotModel.getHumanoidRobotKinematicsCollisionModel());
      FullHumanoidRobotModel fullRobotModelAtInitialConfiguration = createFullRobotModelAtInitialConfiguration(robotModel);
      toolboxController.updateRobotConfigurationData(extractRobotConfigurationData(fullRobotModelAtInitialConfiguration));
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(fullRobotModelAtInitialConfiguration, robotModel, true, true));

      List<Collidable> collidables = robotModel.getHumanoidRobotKinematicsCollisionModel().getRobotCollidables(desiredFullRobotModel.getElevator());

      assertTrue(toolboxController.initialize());
      snapSCSRobotToFullRobotModel(toolboxController.getDesiredFullRobotModel(), robot);

      if (visualize)
         simulationTestHelper.simulateOneTickNow();

      double circleRadius = 0.25;
      double circleFrequency = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.2, side.negateIfRightSide(0.225), 1.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ?
            new Vector3D(0.0, 0.0, 0.0) :
            new Vector3D());

      double toolboxControllerPeriod = toolboxController.getTools().getToolboxControllerPeriod();

      for (double t = 0.0; t < 30.0; t += toolboxControllerPeriod)
      {
         KinematicsStreamingToolboxInputMessage input = new KinematicsStreamingToolboxInputMessage();
         input.getInputs().add().set(KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(fullRobotModelAtInitialConfiguration.getPelvis()));

         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint3D position = circlePositionAt(t,
                                                     robotSide.negateIfRightSide(circleFrequency),
                                                     circleRadius,
                                                     circleCenters.get(robotSide),
                                                     circleCenterVelocities.get(robotSide));
            KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(desiredFullRobotModel.getHand(robotSide),
                                                                                                             position);
            input.getInputs().add().set(message);
         }

         commandInputManager.submitMessage(input);
         toolboxController.update();
         snapSCSRobotToFullRobotModel(desiredFullRobotModel, robot);

         if (visualize)
            simulationTestHelper.simulateOneTickNow();

         for (int collidable1Index = 0; collidable1Index < collidables.size(); collidable1Index++)
         {
            Collidable collidable1 = collidables.get(collidable1Index);

            for (int collidable2Index = 0; collidable2Index < collidables.size(); collidable2Index++)
            {
               Collidable collidable2 = collidables.get(collidable2Index);

               if (collidable1.isCollidableWith(collidable2))
               {
                  CollisionResult collision = collidable1.evaluateCollision(collidable2);
                  EuclidFrameShape3DCollisionResult collisionData = collision.getCollisionData();
                  assertTrue(collisionData.getSignedDistance() > -1.5e-3,
                             collidable1.getRigidBody().getName() + ", " + collidable2.getRigidBody().getName() + ": " + collisionData.getSignedDistance());
               }
            }
         }
      }
   }

   protected void wakeupToolbox()
   {
      ToolboxStateMessage wakeupMessage = new ToolboxStateMessage();
      wakeupMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      statePublisher.publish(wakeupMessage);
   }

   protected void sleepToolbox()
   {
      ToolboxStateMessage wakeupMessage = new ToolboxStateMessage();
      wakeupMessage.setRequestedToolboxState(ToolboxState.SLEEP.toByte());
      statePublisher.publish(wakeupMessage);
   }

   protected ScheduledFuture<?> scheduleMessageGenerator(double dt, DoubleFunction<KinematicsStreamingToolboxInputMessage> messageGenerator)
   {
      if (executor == null)
         executor = ThreadTools.newSingleDaemonThreadScheduledExecutor("inputs-generator");

      return executor.scheduleAtFixedRate(new Runnable()
      {
         double time = 0.0;

         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;
            inputPublisher.publish(messageGenerator.apply(time));
            time += dt;
         }
      }, 0, (int) (dt * 1000), TimeUnit.MILLISECONDS);
   }

   @Test
   public void testStreamingToController()
   {
      testStreamingToController(0.15, 0.25);
   }

   public void testStreamingToController(double handPositionMeanErrorThreshold, double handOrientationMeanErrorThreshold)
   {
      testStreamingToController(new IKStreamingTestRunParameters(null,
                                                                 handPositionMeanErrorThreshold,
                                                                 handOrientationMeanErrorThreshold,
                                                                 0.01,
                                                                 circleMessageGenerator(newRobotModel().createFullRobotModel(), true, 0.125)));
   }

   public void testStreamingToController(IKStreamingTestRunParameters IKStreamingTestRunParameters)
   {
      YoRegistry spyRegistry = new YoRegistry("spy");
      YoDouble handPositionMeanError = new YoDouble("HandsPositionMeanError", spyRegistry);
      YoDouble handOrientationMeanError = new YoDouble("HandsOrientationMeanError", spyRegistry);

      setupWithWalkingController(IKStreamingTestRunParameters.toolboxParameters(), new Controller()
      {
         private final SideDependentList<YoFramePose3D> handDesiredPoses = new SideDependentList<>(side -> new YoFramePose3D(
               side.getCamelCaseName() + "HandDesired", worldFrame, spyRegistry));
         private final SideDependentList<YoFramePose3D> handCurrentPoses = new SideDependentList<>(side -> new YoFramePose3D(
               side.getCamelCaseName() + "HandCurrent", worldFrame, spyRegistry));
         private final SideDependentList<YoDouble> handPositionErrors = new SideDependentList<>(side -> new YoDouble(
               side.getCamelCaseName() + "HandPositionError", spyRegistry));
         private final SideDependentList<YoDouble> handOrientationErrors = new SideDependentList<>(side -> new YoDouble(
               side.getCamelCaseName() + "HandOrientationError", spyRegistry));
         private YoDouble time;
         private YoBoolean isStreaming;
         private YoDouble streamingStartTime;
         private YoDouble streamingBlendingDuration;
         private YoDouble mainStateMachineSwitchTime;
         private YoEnum<KSTState> mainStateMachineCurrentState;

         private boolean needsToInitialize = true;

         @SuppressWarnings("unchecked")
         @Override
         public void initialize()
         {
            if (!needsToInitialize)
               return;

            time = (YoDouble) toolboxRegistry.findVariable("time");
            isStreaming = (YoBoolean) toolboxRegistry.findVariable("isStreaming");
            streamingStartTime = (YoDouble) toolboxRegistry.findVariable("streamingStartTime");
            streamingBlendingDuration = (YoDouble) toolboxRegistry.findVariable("streamingBlendingDuration");
            mainStateMachineSwitchTime = (YoDouble) toolboxRegistry.findVariable("mainStateMachineSwitchTime");
            mainStateMachineCurrentState = (YoEnum<KSTState>) toolboxRegistry.findVariable("mainStateMachineCurrentState");

            needsToInitialize = false;
         }

         private final Mean positionMean = new Mean();
         private final Mean orientationMean = new Mean();

         @Override
         public void doControl()
         {
            initialize();

            if (mainStateMachineCurrentState.getEnumValue() != KSTState.STREAMING || !isStreaming.getValue())
            {
               handDesiredPoses.values().forEach(YoFramePose3D::setToNaN);
               handCurrentPoses.values().forEach(YoFramePose3D::setToNaN);
               return;
            }

            double timeInStream = time.getValue() - mainStateMachineSwitchTime.getValue() - streamingStartTime.getValue();

            if (timeInStream < streamingBlendingDuration.getValue() + 10.0 * toolboxControllerPeriod)
            {
               handDesiredPoses.values().forEach(YoFramePose3D::setToNaN);
               handCurrentPoses.values().forEach(YoFramePose3D::setToNaN);
               return;
            }

            for (RobotSide robotSide : RobotSide.values)
            {
               YoFramePose3D handDesiredPose = handDesiredPoses.get(robotSide);
               YoFramePose3D handCurrentPose = handCurrentPoses.get(robotSide);
               YoDouble handPositionError = handPositionErrors.get(robotSide);
               YoDouble handOrientationError = handOrientationErrors.get(robotSide);

               handDesiredPose.setFromReferenceFrame(desiredFullRobotModel.getHandControlFrame(robotSide));
               handCurrentPose.setFromReferenceFrame(toolboxController.getTools().getCurrentFullRobotModel().getHandControlFrame(robotSide));
               handPositionError.set(handDesiredPose.getPositionDistance(handCurrentPose));
               handOrientationError.set(handDesiredPose.getOrientationDistance(handCurrentPose));
               positionMean.increment(handPositionError.getValue());
               orientationMean.increment(handOrientationError.getValue());
            }

            handPositionMeanError.set(positionMean.getResult());
            handOrientationMeanError.set(orientationMean.getResult());
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return spyRegistry;
         }
      });

      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      wakeupToolbox();

      ScheduledFuture<?> scheduleMessageGenerator = scheduleMessageGenerator(IKStreamingTestRunParameters.messageGeneratorDT(),
                                                                             IKStreamingTestRunParameters.messageGenerator());

      success = simulationTestHelper.simulateNow(10.0);
      assertTrue(success);

      scheduleMessageGenerator.cancel(true);

      KinematicsStreamingToolboxInputMessage message = new KinematicsStreamingToolboxInputMessage();
      message.setStreamToController(false);
      inputPublisher.publish(message);

      sleepToolbox();

      executor.shutdownNow();

      // Asserts that the spy did run and that the toolbox or something did not just hang
      assertNotEquals(0.0, handPositionMeanError.getValue());
      assertNotEquals(0.0, handOrientationMeanError.getValue());
      // TODO Pretty bad assertions here, need to figure out how to improve this test later.
      System.out.println("Position error avg: " + handPositionMeanError.getValue() + ", orientation error avg: " + handOrientationMeanError.getValue());
      assertTrue(handPositionMeanError.getValue() < IKStreamingTestRunParameters.handPositionMeanErrorThreshold(),
                 "Mean position error is: " + handPositionMeanError.getValue());
      assertTrue(handOrientationMeanError.getValue() < IKStreamingTestRunParameters.handOrientationMeanErrorThreshold(),
                 "Mean orientation error is: " + handOrientationMeanError.getValue());
   }

   public static DoubleFunction<KinematicsStreamingToolboxInputMessage> circleMessageGenerator(FullHumanoidRobotModel fullRobotModel,
                                                                                               boolean streamToController,
                                                                                               double frequency)
   {
      double circleRadius = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.3, side.negateIfRightSide(0.225), 1.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ?
            new Vector3D(0.0, 0.0, 0.0) :
            new Vector3D());

      return new DoubleFunction<KinematicsStreamingToolboxInputMessage>()
      {
         @Override
         public KinematicsStreamingToolboxInputMessage apply(double time)
         {
            KinematicsStreamingToolboxInputMessage input = new KinematicsStreamingToolboxInputMessage();
            input.setStreamToController(streamToController);

            for (RobotSide robotSide : RobotSide.values)
            {
               FramePoint3D position = circlePositionAt(time,
                                                        robotSide.negateIfRightSide(frequency),
                                                        circleRadius,
                                                        circleCenters.get(robotSide),
                                                        circleCenterVelocities.get(robotSide));
               KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(fullRobotModel.getHand(robotSide), position);
               input.getInputs().add().set(message);
            }
            return input;
         }
      };
   }

   protected static void hideRobot(Robot robot)
   {
      robot.getFloatingRootJoint().getJointPose().getPosition().set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   }

   protected static void snapSCSRobotToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel, Robot robotToSnap)
   {
      MultiBodySystemTools.copyJointsState(fullHumanoidRobotModel.getRootJoint().subtreeList(),
                                           robotToSnap.getFloatingRootJoint().subtreeList(),
                                           JointStateType.CONFIGURATION);
      robotToSnap.updateFrames();
   }

   public static void copyFullRobotModelState(FullHumanoidRobotModel source, FullHumanoidRobotModel destination)
   {
      for (JointStateType stateSelection : JointStateType.values())
         MultiBodySystemTools.copyJointsState(source.getRootJoint().subtreeList(), destination.getRootJoint().subtreeList(), stateSelection);
   }

   public static List<VisualDefinition> getCollisionVisuals(Collidable collidable)
   {
      Shape3DReadOnly shape = collidable.getShape();
      RigidBodyTransform transformToParentJoint = collidable.getShape()
                                                            .getReferenceFrame()
                                                            .getTransformToDesiredFrame(collidable.getRigidBody().getParentJoint().getFrameAfterJoint());
      VisualDefinitionFactory visualFactory = new VisualDefinitionFactory();
      visualFactory.appendTransform(transformToParentJoint);
      visualFactory.setDefaultMaterial(ColorDefinitions.DarkGreen().derive(0, 1, 1, 0.5));
      visualFactory.addShape(shape);
      return visualFactory.getVisualDefinitions();
   }

   public static final class IKStreamingTestRunParameters
   {
      private KinematicsStreamingToolboxParameters toolboxParameters = KinematicsStreamingToolboxParameters.defaultParameters();
      private double handPositionMeanErrorThreshold = 0.15;
      private double handOrientationMeanErrorThreshold = 0.25;
      private double messageGeneratorDT = 0.01;
      private DoubleFunction<KinematicsStreamingToolboxInputMessage> messageGenerator;

      public IKStreamingTestRunParameters(KinematicsStreamingToolboxParameters toolboxParameters,
                                          double handPositionMeanErrorThreshold,
                                          double handOrientationMeanErrorThreshold,
                                          double messageGeneratorDT,
                                          DoubleFunction<KinematicsStreamingToolboxInputMessage> messageGenerator)
      {
         this.toolboxParameters = toolboxParameters;
         this.handPositionMeanErrorThreshold = handPositionMeanErrorThreshold;
         this.handOrientationMeanErrorThreshold = handOrientationMeanErrorThreshold;
         this.messageGeneratorDT = messageGeneratorDT;
         this.messageGenerator = messageGenerator;
      }

      public void setToolboxParameters(KinematicsStreamingToolboxParameters toolboxParameters)
      {
         this.toolboxParameters = toolboxParameters;
      }

      public KinematicsStreamingToolboxParameters toolboxParameters()
      {
         return toolboxParameters;
      }

      public double handPositionMeanErrorThreshold()
      {
         return handPositionMeanErrorThreshold;
      }

      public double handOrientationMeanErrorThreshold()
      {
         return handOrientationMeanErrorThreshold;
      }

      public double messageGeneratorDT()
      {
         return messageGeneratorDT;
      }

      public DoubleFunction<KinematicsStreamingToolboxInputMessage> messageGenerator()
      {
         return messageGenerator;
      }

      @Override
      public boolean equals(Object obj)
      {
         if (obj == this)
            return true;
         if (obj == null || obj.getClass() != this.getClass())
            return false;
         var that = (IKStreamingTestRunParameters) obj;
         return Objects.equals(this.toolboxParameters, that.toolboxParameters)
                && Double.doubleToLongBits(this.handPositionMeanErrorThreshold) == Double.doubleToLongBits(that.handPositionMeanErrorThreshold)
                && Double.doubleToLongBits(this.handOrientationMeanErrorThreshold) == Double.doubleToLongBits(that.handOrientationMeanErrorThreshold)
                && Double.doubleToLongBits(this.messageGeneratorDT) == Double.doubleToLongBits(that.messageGeneratorDT) && Objects.equals(this.messageGenerator,
                                                                                                                                          that.messageGenerator);
      }

      @Override
      public int hashCode()
      {
         return Objects.hash(toolboxParameters, handPositionMeanErrorThreshold, handOrientationMeanErrorThreshold, messageGeneratorDT, messageGenerator);
      }

      @Override
      public String toString()
      {
         return "IKStreamingTestRunParameters[" + "toolboxParameters=" + toolboxParameters + ", " + "handPositionMeanErrorThreshold="
                + handPositionMeanErrorThreshold + ", " + "handOrientationMeanErrorThreshold=" + handOrientationMeanErrorThreshold + ", "
                + "messageGeneratorDT=" + messageGeneratorDT + ", " + "messageGenerator=" + messageGenerator + ']';
      }
   }
}