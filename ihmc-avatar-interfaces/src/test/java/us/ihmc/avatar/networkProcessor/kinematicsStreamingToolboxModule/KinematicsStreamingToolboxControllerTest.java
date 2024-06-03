package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import javafx.application.Platform;
import javafx.scene.control.Button;
import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.IKRobotStateUpdater;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters.ClockType;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.WalkingCommandConsumer;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.ControllerAPI;
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
import us.ihmc.ros2.ROS2PublisherBasics;
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
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.Session.SessionModeChangeListener;
import us.ihmc.scs2.session.SessionMode;
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

import java.lang.reflect.Method;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.*;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.RelativeEndEffectorControlTest.circlePositionAt;

@Tag("humanoid-toolbox")
public abstract class KinematicsStreamingToolboxControllerTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
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
   protected ROS2PublisherBasics<KinematicsStreamingToolboxInputMessage> inputPublisher;
   protected ROS2PublisherBasics<ToolboxStateMessage> statePublisher;
   protected ROS2Topic<?> controllerInputTopic;
   protected ROS2Topic<?> controllerOutputTopic;
   protected ROS2Topic<?> toolboxInputTopic;
   protected ROS2Topic<?> toolboxOutputTopic;

   /**
    * Returns a <b>new</b> instance of the robot model that will be modified in this test to create
    * ghost robots.
    */
   public abstract DRCRobotModel newRobotModel();

   public void setupWithWalkingController(IKStreamingTestRunParameters ikStreamingTestRunParameters)
   {
      KinematicsStreamingToolboxParameters toolboxParameters = ikStreamingTestRunParameters.toolboxParameters();
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
      YoBoolean isAutomaticManipulationAbortEnabled = (YoBoolean) simulationTestHelper.getControllerRegistry()
                                                                                      .findVariable(WalkingCommandConsumer.class.getSimpleName(),
                                                                                                    "isAutomaticManipulationAbortEnabled");
      isAutomaticManipulationAbortEnabled.set(false); // TODO This is a hack to prevent the walking controller from aborting the manipulation task.
      createToolboxController(robotModel, toolboxParameters, collisionModel);
      simulationTestHelper.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      ros2Node = simulationTestHelper.getROS2Node();

      controllerInputTopic = HumanoidControllerAPI.getInputTopic(robotName);
      controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      toolboxInputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
      toolboxOutputTopic = KinematicsStreamingToolboxModule.getOutputTopic(robotName);

      RealtimeROS2Node toolboxROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS, "toolbox_node");
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(toolboxInputTopic,
                                                                                                commandInputManager,
                                                                                                toolboxOutputTopic,
                                                                                                statusOutputManager,
                                                                                                toolboxROS2Node);

      ROS2PublisherBasics<WholeBodyTrajectoryMessage> trajectoryOutputPublisher = ros2Node.createPublisher(ControllerAPI.getTopic(controllerInputTopic,
                                                                                                                                  WholeBodyTrajectoryMessage.class));
      toolboxController.setTrajectoryMessagePublisher(trajectoryOutputPublisher::publish);
      ROS2PublisherBasics<WholeBodyStreamingMessage> streamingOutputPublisher = ros2Node.createPublisher(ControllerAPI.getTopic(controllerInputTopic,
                                                                                                                                WholeBodyStreamingMessage.class));
      toolboxController.setStreamingMessagePublisher(streamingOutputPublisher::publish);

      RobotConfigurationDataBasedUpdater robotStateUpdater = new RobotConfigurationDataBasedUpdater();
      toolboxController.setRobotStateUpdater(robotStateUpdater);
      toolboxROS2Node.createSubscription(StateEstimatorAPI.getRobotConfigurationDataTopic(robotName),
                                         s -> robotStateUpdater.setRobotConfigurationData(s.takeNextData()));

      toolboxROS2Node.createSubscription(ControllerAPI.getTopic(controllerOutputTopic, CapturabilityBasedStatus.class),
                                         s -> toolboxController.updateCapturabilityBasedStatus(s.takeNextData()));

      inputPublisher = ros2Node.createPublisher(ControllerAPI.getTopic(toolboxInputTopic, KinematicsStreamingToolboxInputMessage.class));
      statePublisher = ros2Node.createPublisher(toolboxInputTopic.withTypeName(ToolboxStateMessage.class));

      AtomicReference<KinematicsToolboxOutputStatus> toolboxViz = new AtomicReference<>(null);
      ros2Node.createSubscription(ControllerAPI.getTopic(toolboxOutputTopic, KinematicsToolboxOutputStatus.class), s -> toolboxViz.set(s.takeNextData()));

      Controller toolboxUpdater = new Controller()
      {
         private final JointReadOnly[] desiredJoints = MultiBodySystemTools.collectSubtreeJoints(desiredFullRobotModel.getElevator());
         private final ControllerOutputBasics scsInput = ghost.getControllerOutput();

         @Override
         public void doControl()
         {
            try
            {
               toolboxController.update();

               if (ikStreamingTestRunParameters.getIKStreamingAssertion() != null)
               {
                  ikStreamingTestRunParameters.getIKStreamingAssertion().assertExecution(toolboxController);
               }

               for (JointReadOnly joint : desiredJoints)
               {
                  scsInput.getJointOutput(joint).setConfiguration(joint);
               }
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return toolboxRegistry;
         }
      };
      ghost.addThrottledController(toolboxUpdater, toolboxParameters.getToolboxUpdatePeriod());

      toolboxROS2Node.spin();
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
         scs.setDT(toolboxController.getTools().getToolboxControllerPeriod());
         scs.initializeBufferRecordTickPeriod(1);

         simulationTestHelper = new SCS2AvatarTestingSimulation(scs, robotModel, desiredFullRobotModel, yoGraphicsListRegistry, simulationTestingParameters);
         simulationTestHelper.setKeepSCSUp(simulationTestingParameters.getKeepSCSUp());
         simulationTestHelper.start(false);
         simulationTestHelper.setCamera(new Point3D(0, 0, 1), new Point3D(6, 0, 1));
      }
   }

   public static void addCollisionVisuals(FullHumanoidRobotModelFactory fullRobotModelFactory,
                                          RobotCollisionModel collisionModel,
                                          RobotDefinition robotDefinition)
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
      {
         toolboxParameters = KinematicsStreamingToolboxParameters.defaultParameters();
      }

      toolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                   statusOutputManager,
                                                                   toolboxParameters,
                                                                   desiredFullRobotModel,
                                                                   robotModel,
                                                                   yoGraphicsListRegistry,
                                                                   toolboxRegistry);
      toolboxController.setCollisionModel(collisionModel);
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
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
      toolboxController.setRobotStateUpdater(IKRobotStateUpdater.wrap(extractRobotConfigurationData(fullRobotModelAtInitialConfiguration)));
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(fullRobotModelAtInitialConfiguration, robotModel, true, true));

      List<Collidable> collidables = robotModel.getHumanoidRobotKinematicsCollisionModel().getRobotCollidables(desiredFullRobotModel.getElevator());

      assertTrue(toolboxController.initialize());
      snapSCSRobotToFullRobotModel(toolboxController.getDesiredFullRobotModel(), robot);

      if (visualize)
         simulationTestHelper.simulateOneTickNow();

      double circleRadius = 0.25;
      double circleFrequency = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.2, side.negateIfRightSide(0.225), 1.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ? new Vector3D(0.0, 0.0, 0.0)
                                                                                                                  : new Vector3D());

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

   @Test
   public void testStreamingToController()
   {
      testStreamingToController(0.15, 0.25);
   }

   public void testStreamingToController(double handPositionMeanErrorThreshold, double handOrientationMeanErrorThreshold)
   {
      IKStreamingTestRunParameters testRunParameters = new IKStreamingTestRunParameters();
      testRunParameters.setHandPositionMeanErrorThreshold(handPositionMeanErrorThreshold);
      testRunParameters.setHandOrientationMeanErrorThreshold(handOrientationMeanErrorThreshold);
      testRunParameters.setMessageGeneratorDT(0.01);
      testRunParameters.setMessageGenerator(circleMessageGenerator(newRobotModel().createFullRobotModel(), true, 0.125));
      testStreamingToController(testRunParameters);
   }

   public void testStreamingToController(IKStreamingTestRunParameters ikStreamingTestRunParameters)
   {
      setupWithWalkingController(ikStreamingTestRunParameters);
      SpyTrackingController spyController = new SpyTrackingController(toolboxRegistry,
                                                                      toolboxController.getDesiredFullRobotModel(),
                                                                      simulationTestHelper.getControllerFullRobotModel());
      if (ghost != null)
         ghost.addThrottledController(spyController, toolboxController.getTools().getToolboxControllerPeriod());

      if (ikStreamingTestRunParameters.getRegistry() != null)
         simulationTestHelper.addRegistry(ikStreamingTestRunParameters.getRegistry());

      IKStreamingMessageGenerator ikStreamingMessageGenerator = ikStreamingTestRunParameters.messageGenerator();

      ghost.addThrottledController(new Controller()
      {
         @Override
         public void initialize()
         {
            try
            {
               ikStreamingMessageGenerator.initialize();
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }

         @Override
         public void doControl()
         {
            try
            {
               double time = simulationTestHelper.getSimulationTime();
               KinematicsStreamingToolboxInputMessage message = ikStreamingMessageGenerator.update(time);
               if (message != null)
                  inputPublisher.publish(message);
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      }, ikStreamingTestRunParameters.messageGeneratorDT());

      simulationTestHelper.start();
      SimRunner simRunner = new SimRunner(simulationTestHelper);

      SimulationConstructionSet2 scs = simulationTestHelper.getSimulationConstructionSet();
      scs.waitUntilVisualizerFullyUp();
      Platform.runLater(() ->
      {
         Button restart = new Button("Restart");
         restart.setOnAction(event -> simRunner.reset());
         scs.addCustomGUIControl(restart);
      });

      assertTrue(simRunner.simulateNow(0.5));
      wakeupToolbox();

      assertTrue(simRunner.simulateNow(ikStreamingTestRunParameters.simulationDuration()));

      KinematicsStreamingToolboxInputMessage message = new KinematicsStreamingToolboxInputMessage();
      message.setStreamToController(false);
      inputPublisher.publish(message);

      sleepToolbox();

      // Asserts that the spy did run and that the toolbox or something did not just hang
      double handPositionMeanError = spyController.getHandPositionMeanError();
      double handOrientationMeanError = spyController.getHandOrientationMeanError();
      assertNotEquals(0.0, handPositionMeanError);
      assertNotEquals(0.0, handOrientationMeanError);
      // TODO Pretty bad assertions here, need to figure out how to improve this test later.
      System.out.println("Position error avg: " + handPositionMeanError + ", orientation error avg: " + handOrientationMeanError);
      assertTrue(handPositionMeanError < ikStreamingTestRunParameters.handPositionMeanErrorThreshold(), "Mean position error is: " + handPositionMeanError);
      assertTrue(handOrientationMeanError < ikStreamingTestRunParameters.handOrientationMeanErrorThreshold(),
                 "Mean orientation error is: " + handOrientationMeanError);
   }

   public static class SimRunner
   {
      private final SCS2AvatarTestingSimulation simulationTestHelper;

      public SimRunner(SCS2AvatarTestingSimulation simulationTestHelper)
      {
         this.simulationTestHelper = simulationTestHelper;
      }

      private final AtomicBoolean resetSimulation = new AtomicBoolean(false);

      public void reset()
      {
         resetSimulation.set(true);
      }

      public boolean simulateNow(double duration)
      {
         return simulateNow((int) (duration / simulationTestHelper.getSimulationDT()));
      }

      public boolean simulateNow(long numberOfTicks)
      {
         SimulationConstructionSet2 scs = simulationTestHelper.getSimulationConstructionSet();
         SimulationSession session = scs.getSimulationSession();

         if (session.isSessionShutdown())
            return false;

         boolean sessionStartedInitialValue = scs.isSimulationThreadRunning();

         if (sessionStartedInitialValue)
         {
            if (!scs.stopSimulationThread())
               return false;
         }

         SessionMode activeModeInitialValue = session.getActiveMode();
         long maxDurationInitialValue = session.getRunMaxDuration();
         session.submitRunMaxDuration(-1L); // Make sure the max duration does not interfere with the number of ticks.

         try
         {
            session.setSessionMode(SessionMode.RUNNING);

            boolean success = true;

            if (numberOfTicks == -1L || numberOfTicks == Long.MAX_VALUE)
            {
               return scs.simulateNow(numberOfTicks);
            }
            else
            {
               for (long tick = 0; tick < numberOfTicks; tick++)
               {
                  if (session.isSessionShutdown())
                     return false;

                  if (!handleVisualizerSessionModeRequests())
                     break;

                  if (resetSimulation.getAndSet(false))
                  {
                     scs.reinitializeSimulation();
                     tick = 0;
                  }

                  success = session.runTick();

                  if (!success)
                     break;
               }
            }

            return success;
         }
         finally
         {
            // This ensures that the controller is being pause.
            session.getPhysicsEngine().pause();
            session.submitRunMaxDuration(maxDurationInitialValue); // Restore the max duration.
            session.requestBufferListenerForceUpdate();

            if (sessionStartedInitialValue)
               session.startSessionThread();
            session.setSessionMode(activeModeInitialValue);
         }
      }

      private boolean handleVisualizerSessionModeRequests()
      {
         SimulationConstructionSet2 scs = simulationTestHelper.getSimulationConstructionSet();
         SimulationSession session = scs.getSimulationSession();

         if (scs.isSimulating() || !session.hasWrittenBufferInLastRunTick())
            return true; // Make sure we stop running when the buffer was just updated.

         // The GUI requested a mode change, we pause the simulation until the GUI request RUNNING again.
         CountDownLatch latch = new CountDownLatch(1);

         SessionModeChangeListener listener = (prevMode, newMode) ->
         {
            if (newMode != prevMode && newMode == SessionMode.RUNNING)
            {
               scs.stopSimulationThread();
               if (scs.getBufferCurrentIndex() != scs.getBufferOutPoint())
               { // We make sure to go back to the out-point
                  scs.gotoBufferOutPoint();
                  finalizePauseTick();
               }
               latch.countDown();
            }
         };
         session.addPreSessionModeChangeListener(listener);

         session.startSessionThread();

         try
         {
            latch.await();
         }
         catch (InterruptedException e)
         {
            return false;
         }
         finally
         {
            session.removePreSessionModeChangeListener(listener);
         }

         return true;
      }

      private void finalizePauseTick()
      {
         SimulationConstructionSet2 scs = simulationTestHelper.getSimulationConstructionSet();
         SimulationSession session = scs.getSimulationSession();
         try
         {
            Method finalizePauseTick = Session.class.getDeclaredMethod("finalizePauseTick", boolean.class);
            finalizePauseTick.setAccessible(true);
            finalizePauseTick.invoke(session, true);
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   public static IKStreamingMessageGenerator circleMessageGenerator(FullHumanoidRobotModel fullRobotModel, boolean streamToController, double frequency)
   {
      double circleRadius = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.3, side.negateIfRightSide(0.225), 1.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ? new Vector3D(0.0, 0.0, 0.0)
                                                                                                                  : new Vector3D());

      return time ->
      {
         KinematicsStreamingToolboxInputMessage input = new KinematicsStreamingToolboxInputMessage();
         input.setStreamToController(streamToController);

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
            if (hand == null)
               continue;
            FramePoint3D position = circlePositionAt(time,
                                                     robotSide.negateIfRightSide(frequency),
                                                     circleRadius,
                                                     circleCenters.get(robotSide),
                                                     circleCenterVelocities.get(robotSide));
            KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, position);
            input.getInputs().add().set(message);
         }
         return input;
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
      private IKStreamingMessageGenerator messageGenerator;
      private IKStreamingAssertion ikStreamingAssertion;
      private double simulationDuration = 10.0;

      private YoRegistry registry;

      public IKStreamingTestRunParameters()
      {
         toolboxParameters.setClockType(ClockType.FIXED_DT);
      }

      public void setToolboxParameters(KinematicsStreamingToolboxParameters toolboxParameters)
      {
         this.toolboxParameters = toolboxParameters;
      }

      public KinematicsStreamingToolboxParameters toolboxParameters()
      {
         return toolboxParameters;
      }

      public void setHandPositionMeanErrorThreshold(double handPositionMeanErrorThreshold)
      {
         this.handPositionMeanErrorThreshold = handPositionMeanErrorThreshold;
      }

      public double handPositionMeanErrorThreshold()
      {
         return handPositionMeanErrorThreshold;
      }

      public void setHandOrientationMeanErrorThreshold(double handOrientationMeanErrorThreshold)
      {
         this.handOrientationMeanErrorThreshold = handOrientationMeanErrorThreshold;
      }

      public double handOrientationMeanErrorThreshold()
      {
         return handOrientationMeanErrorThreshold;
      }

      public void setMessageGeneratorDT(double messageGeneratorDT)
      {
         this.messageGeneratorDT = messageGeneratorDT;
      }

      public double messageGeneratorDT()
      {
         return messageGeneratorDT;
      }

      public void setMessageGenerator(IKStreamingMessageGenerator messageGenerator)
      {
         this.messageGenerator = messageGenerator;
      }

      public IKStreamingMessageGenerator messageGenerator()
      {
         return messageGenerator;
      }

      public void setIKStreamingAssertion(IKStreamingAssertion ikStreamingAssertion)
      {
         this.ikStreamingAssertion = ikStreamingAssertion;
      }

      public IKStreamingAssertion getIKStreamingAssertion()
      {
         return ikStreamingAssertion;
      }

      public void setSimulationDuration(double simulationDuration)
      {
         this.simulationDuration = simulationDuration;
      }

      public double simulationDuration()
      {
         return simulationDuration;
      }

      public void setRegistry(YoRegistry registry)
      {
         this.registry = registry;
      }

      public YoRegistry getRegistry()
      {
         return registry;
      }

      @Override
      public String toString()
      {
         return "[toolboxParameters=%s, handPositionMeanErrorThreshold=%s, handOrientationMeanErrorThreshold=%s, messageGeneratorDT=%s, messageGenerator=%s, simulationDuration=%s]".formatted(toolboxParameters,
                                                                                                                                                                                               handPositionMeanErrorThreshold,
                                                                                                                                                                                               handOrientationMeanErrorThreshold,
                                                                                                                                                                                               messageGeneratorDT,
                                                                                                                                                                                               messageGenerator,
                                                                                                                                                                                               simulationDuration);
      }
   }

   public interface IKStreamingMessageGenerator
   {
      default void initialize()
      {
      }

      KinematicsStreamingToolboxInputMessage update(double time);
   }

   public interface IKStreamingAssertion
   {
      void assertExecution(KinematicsStreamingToolboxController toolboxController);
   }

   public static IKStreamingMessageGenerator blop()
   {
      IKStreamingMessageGenerator generator = time ->
      {
         KinematicsStreamingToolboxInputMessage ret = new KinematicsStreamingToolboxInputMessage();
         // des truc
         return ret;
      };
      return generator;
   }

   public static class SpyTrackingController implements Controller
   {
      private final YoRegistry toolboxRegistry;
      private final FullHumanoidRobotModel desiredFullRobotModel;
      private final FullHumanoidRobotModel currentFullRobotModel;
      private final YoRegistry spyRegistry = new YoRegistry("spy");
      private final YoDouble handPositionMeanError = new YoDouble("HandsPositionMeanError", spyRegistry);
      private final YoDouble handOrientationMeanError = new YoDouble("HandsOrientationMeanError", spyRegistry);

      private final SideDependentList<YoFramePose3D> handDesiredPoses = new SideDependentList<>();
      private final SideDependentList<YoFramePose3D> handCurrentPoses = new SideDependentList<>();
      private final SideDependentList<YoDouble> handPositionErrors = new SideDependentList<>();
      private final SideDependentList<YoDouble> handOrientationErrors = new SideDependentList<>();
      private YoDouble time;
      private YoBoolean isStreaming;
      private YoDouble streamingStartTime;
      private YoDouble streamingBlendingDuration;
      private YoDouble mainStateMachineSwitchTime;
      private YoEnum<KSTState> mainStateMachineCurrentState;

      private boolean needsToInitialize;

      public SpyTrackingController(YoRegistry toolboxRegistry, FullHumanoidRobotModel desiredFullRobotModel, FullHumanoidRobotModel currentFullRobotModel)
      {
         this.toolboxRegistry = toolboxRegistry;
         this.desiredFullRobotModel = desiredFullRobotModel;
         this.currentFullRobotModel = currentFullRobotModel;

         for (RobotSide side : RobotSide.values)
         {
            if (desiredFullRobotModel.getHand(side) == null)
               continue;

            handDesiredPoses.put(side, new YoFramePose3D(side.getCamelCaseName() + "HandDesired", worldFrame, spyRegistry));
            handCurrentPoses.put(side, new YoFramePose3D(side.getCamelCaseName() + "HandCurrent", worldFrame, spyRegistry));
            handPositionErrors.put(side, new YoDouble(side.getCamelCaseName() + "HandPositionError", spyRegistry));
            handOrientationErrors.put(side, new YoDouble(side.getCamelCaseName() + "HandOrientationError", spyRegistry));
         }
         needsToInitialize = true;
         positionMean = new Mean();
         orientationMean = new Mean();
      }

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

      private final Mean positionMean;
      private final Mean orientationMean;

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

         if (timeInStream < streamingBlendingDuration.getValue() + 0.1)
         {
            handDesiredPoses.values().forEach(YoFramePose3D::setToNaN);
            handCurrentPoses.values().forEach(YoFramePose3D::setToNaN);
            return;
         }

         for (RobotSide robotSide : RobotSide.values)
         {
            if (desiredFullRobotModel.getHand(robotSide) == null)
               continue;

            YoFramePose3D handDesiredPose = handDesiredPoses.get(robotSide);
            YoFramePose3D handCurrentPose = handCurrentPoses.get(robotSide);
            YoDouble handPositionError = handPositionErrors.get(robotSide);
            YoDouble handOrientationError = handOrientationErrors.get(robotSide);

            handDesiredPose.setFromReferenceFrame(desiredFullRobotModel.getHandControlFrame(robotSide));
            handCurrentPose.setFromReferenceFrame(currentFullRobotModel.getHandControlFrame(robotSide));
            handPositionError.set(handDesiredPose.getPositionDistance(handCurrentPose));
            handOrientationError.set(handDesiredPose.getOrientationDistance(handCurrentPose));
            positionMean.increment(handPositionError.getValue());
            orientationMean.increment(handOrientationError.getValue());
         }

         handPositionMeanError.set(positionMean.getResult());
         handOrientationMeanError.set(orientationMean.getResult());
      }

      public double getHandPositionMeanError()
      {
         return handPositionMeanError.getValue();
      }

      public double getHandOrientationMeanError()
      {
         return handOrientationMeanError.getValue();
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return spyRegistry;
      }
   }
}