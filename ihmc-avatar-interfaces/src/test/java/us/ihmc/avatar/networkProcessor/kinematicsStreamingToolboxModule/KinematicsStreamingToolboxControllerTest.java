package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.createFullRobotModelAtInitialConfiguration;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.RelativeEndEffectorControlTest.circlePositionAt;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleFunction;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollisionResult;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

@Tag("humanoid-toolbox")
public abstract class KinematicsStreamingToolboxControllerTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected static final double toolboxControllerPeriod = 5.0e-3;
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
   protected static final boolean visualize = simulationTestingParameters.getCreateGUI();
   static
   {
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   protected CommandInputManager commandInputManager;
   protected StatusMessageOutputManager statusOutputManager;
   protected YoRegistry toolboxRegistry;
   protected YoGraphicsListRegistry yoGraphicsListRegistry;
   protected FullHumanoidRobotModel desiredFullRobotModel;
   protected KinematicsStreamingToolboxController toolboxController;

   protected SimulationConstructionSet scs;
   protected DRCSimulationTestHelper drcSimulationTestHelper;
   protected HumanoidFloatingRootJointRobot robot, ghost;
   protected Ros2Node ros2Node;
   protected IHMCROS2Publisher<KinematicsStreamingToolboxInputMessage> inputPublisher;
   protected IHMCROS2Publisher<ToolboxStateMessage> statePublisher;
   protected ROS2Topic controllerInputTopic;
   protected ROS2Topic controllerOutputTopic;
   protected ROS2Topic toolboxInputTopic;
   protected ROS2Topic toolboxOutputTopic;
   protected ScheduledExecutorService executor;

   /**
    * Returns a <b>new</b> instance of the robot model that will be modified in this test to create
    * ghost robots.
    */
   public abstract DRCRobotModel newRobotModel();

   public void setupWithWalkingController(RobotController... additionalGhostControllers)
   {
      DRCRobotModel ghostRobotModel = newRobotModel();
      String robotName = ghostRobotModel.getSimpleRobotName();
      ghost = createSCSRobot(ghostRobotModel, "ghost", ghostApperance);
      hideRobot(ghost);
      ghost.setDynamic(false);
      ghost.setGravity(0);
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      testEnvironment.addEnvironmentRobot(ghost);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, newRobotModel(), testEnvironment);
      createToolboxController(ghostRobotModel);
      setupCollisions(ghostRobotModel.getHumanoidRobotKinematicsCollisionModel(), ghost);

      ros2Node = drcSimulationTestHelper.getRos2Node();

      controllerInputTopic = ROS2Tools.getControllerInputTopic(robotName);
      controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      toolboxInputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
      toolboxOutputTopic = KinematicsStreamingToolboxModule.getOutputTopic(robotName);

      RealtimeRos2Node toolboxRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, "toolbox_node");
      new ControllerNetworkSubscriber(toolboxInputTopic, commandInputManager, toolboxOutputTopic, statusOutputManager, toolboxRos2Node);
      IHMCROS2Publisher<WholeBodyTrajectoryMessage> outputPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                         WholeBodyTrajectoryMessage.class,
                                                                                                         controllerInputTopic);
      toolboxController.setOutputPublisher(outputPublisher::publish);

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

      ghost.setController(new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(ghost, desiredFullRobotModel);

         @Override
         public void initialize()
         {
         }

         @Override
         public void doControl()
         {
            toolboxController.update();

            jointAnglesWriter.setWriteJointVelocities(false);
            jointAnglesWriter.setWriteJointAccelerations(false);
            jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return toolboxRegistry;
         }
      }, (int) (toolboxControllerPeriod / ghostRobotModel.getSimulateDT()));

      if (additionalGhostControllers != null)
      {
         for (RobotController ghostController : additionalGhostControllers)
            ghost.setController(ghostController, (int) (toolboxControllerPeriod / ghostRobotModel.getSimulateDT()));
      }

      toolboxRos2Node.spin();
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      scs = drcSimulationTestHelper.getSimulationConstructionSet();
   }

   public void setupNoWalkingController(RobotCollisionModel collisionModel)
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DRCRobotModel robotModel = newRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      RobotDescription robotDescription = robotModel.getRobotDescription();
      robot = new HumanoidFloatingRootJointRobot(robotDescription, robotModel.getJointMap());
      createToolboxController(robotModel);
      setupCollisions(collisionModel, robot);

      robot.setDynamic(false);
      robot.setGravity(0);

      ghost = createSCSRobot(newRobotModel(), "ghost", ghostApperance);
      hideRobot(ghost);
      ghost.setDynamic(false);
      ghost.setGravity(0);

      if (visualize)
      {
         Robot[] robots = ghost != null ? new Robot[] {robot, ghost} : new Robot[] {robot};
         scs = new SimulationConstructionSet(robots, simulationTestingParameters);
         scs.addYoRegistry(toolboxRegistry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setCameraFix(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
         scs.setDT(toolboxControllerPeriod, 1);
         scs.startOnAThread();
      }
   }

   private void setupCollisions(RobotCollisionModel collisionModel, HumanoidFloatingRootJointRobot robot)
   {
      toolboxController.setCollisionModel(collisionModel);

      if (collisionModel != null)
      {
         List<Collidable> collidables = collisionModel.getRobotCollidables(desiredFullRobotModel.getElevator());

         for (Collidable collidable : collidables)
         {
            Graphics3DObject graphics = getGraphics(collidable);
            Link link = robot.getLink(collidable.getRigidBody().getName());
            Graphics3DObject linkGraphics = link.getLinkGraphics();
            if (linkGraphics == null)
            {
               linkGraphics = new Graphics3DObject();
               link.setLinkGraphics(linkGraphics);
            }
            linkGraphics.combine(graphics);
         }
      }
   }

   private void createToolboxController(DRCRobotModel robotModel)
   {
      desiredFullRobotModel = robotModel.createFullRobotModel();
      toolboxRegistry = new YoRegistry("toolboxMain");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsStreamingToolboxCommandConverter(desiredFullRobotModel));
      statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());

      toolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                   statusOutputManager,
                                                                   desiredFullRobotModel,
                                                                   robotModel,
                                                                   robotModel.getControllerDT(),
                                                                   toolboxControllerPeriod,
                                                                   yoGraphicsListRegistry,
                                                                   toolboxRegistry);
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
         scs = null;
      }
      else if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
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
         scs.tickAndUpdate();
   
      double circleRadius = 0.25;
      double circleFrequency = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.2, side.negateIfRightSide(0.225), 1.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ? new Vector3D(0.0, 0.0, 0.0)
            : new Vector3D());
   
      double toolboxControllerPeriod = toolboxController.getTools().getToolboxControllerPeriod();
   
      for (double t = 0.0; t < 100.0; t += toolboxControllerPeriod)
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
         {
            Arrays.asList(scs.getRobots()).forEach(robot -> robot.getYoTime().add(toolboxControllerPeriod));
            scs.tickAndUpdate();
         }
   
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
                  assertTrue(collisionData.getSignedDistance() > -1.5e-3, collidable1.getRigidBody().getName() + ", " + collidable2.getRigidBody().getName() + ": " + collisionData.getSignedDistance());
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
   public void testStreamingToController() throws SimulationExceededMaximumTimeException
   {
      YoRegistry spyRegistry = new YoRegistry("spy");
      YoDouble handPositionMeanError = new YoDouble("HandsPositionMeanError", spyRegistry);
      YoDouble handOrientationMeanError = new YoDouble("HandsOrientationMeanError", spyRegistry);
   
      setupWithWalkingController(new RobotController()
      {
         private final SideDependentList<YoFramePose3D> handDesiredPoses = new SideDependentList<>(side -> new YoFramePose3D(side.getCamelCaseName()
               + "HandDesired", worldFrame, spyRegistry));
         private final SideDependentList<YoFramePose3D> handCurrentPoses = new SideDependentList<>(side -> new YoFramePose3D(side.getCamelCaseName()
               + "HandCurrent", worldFrame, spyRegistry));
         private final SideDependentList<YoDouble> handPositionErrors = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseName()
               + "HandPositionError", spyRegistry));
         private final SideDependentList<YoDouble> handOrientationErrors = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseName()
               + "HandOrientationError", spyRegistry));
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
   
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
   
      wakeupToolbox();
   
      ScheduledFuture<?> scheduleMessageGenerator = scheduleMessageGenerator(0.01, circleMessageGenerator(newRobotModel().createFullRobotModel(), true, 0.125));
   
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
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
      assertTrue(handPositionMeanError.getValue() < 0.15, "Mean position error is: " + handPositionMeanError.getValue());
      assertTrue(handOrientationMeanError.getValue() < 0.25, "Mean orientation error is: " + handOrientationMeanError.getValue());
   }

   public static DoubleFunction<KinematicsStreamingToolboxInputMessage> circleMessageGenerator(FullHumanoidRobotModel fullRobotModel, boolean streamToController, double frequency)
   {
      double circleRadius = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.3, side.negateIfRightSide(0.225), 1.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ? new Vector3D(0.0, 0.0, 0.0)
            : new Vector3D());
   
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

   protected static HumanoidFloatingRootJointRobot createSCSRobot(DRCRobotModel ghostRobotModel, String robotName, AppearanceDefinition robotAppearance)
   {
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName(robotName);
      KinematicsToolboxControllerTest.recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), robotAppearance);
      HumanoidFloatingRootJointRobot scsRobot = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      scsRobot.getRootJoint().setPinned(true);
      scsRobot.setDynamic(false);
      scsRobot.setGravity(0);
      return scsRobot;
   }

   protected static void hideRobot(HumanoidFloatingRootJointRobot robot)
   {
      robot.setPositionInWorld(new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
   }

   protected static void snapSCSRobotToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel, HumanoidFloatingRootJointRobot robotToSnap)
   {
      JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robotToSnap, fullHumanoidRobotModel);
      jointAnglesWriter.setWriteJointVelocities(false);
      jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
   }

   public static void copyFullRobotModelState(FullHumanoidRobotModel source, FullHumanoidRobotModel destination)
   {
      for (JointStateType stateSelection : JointStateType.values())
         MultiBodySystemTools.copyJointsState(source.getRootJoint().subtreeList(), destination.getRootJoint().subtreeList(), stateSelection);
   }

   public static Graphics3DObject getGraphics(Collidable collidable)
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
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape.getClass().getSimpleName());
      }
      return graphics;
   }
}