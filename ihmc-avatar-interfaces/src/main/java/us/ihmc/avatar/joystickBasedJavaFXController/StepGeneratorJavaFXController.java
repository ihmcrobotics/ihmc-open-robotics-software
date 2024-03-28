package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.SteppingParameters;
import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.StepsAreAdjustable;
import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.WalkingTrajectoryDuration;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.ButtonBState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.ButtonLeftBumperState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.ButtonRightBumperState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.ButtonSelectState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.ButtonStartState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.ButtonXState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.ButtonYState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.DPadDownState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.DPadLeftState;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.LeftStickXAxis;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.LeftStickYAxis;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.RightStickXAxis;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javafx.JavaFXRobotVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class StepGeneratorJavaFXController
{
   private final long footstepPublishingPeriod = TimeUnit.MILLISECONDS.toNanos(100);
   private final AtomicBoolean publishFootstepRequest = new AtomicBoolean(true);
   private final SideDependentList<Color> footColors = new SideDependentList<>(Color.CRIMSON, Color.YELLOWGREEN);

   private final ContinuousStepController continuousStepController;

   private final AnimationTimer animationTimer = new AnimationTimer()
   {
      @Override
      public void handle(long now)
      {
         update(now);
      }
   };
   private final AtomicReference<List<Node>> footstepsToVisualizeReference = new AtomicReference<>(null);

   private final Group rootNode = new Group();

   private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);

   private final AtomicReference<Double> trajectoryDuration;
   private final AtomicReference<Boolean> stepsAreAdjustable;
   private final JavaFXRobotVisualizer javaFXRobotVisualizer;

   public enum SecondaryControlOption
   {
      KICK, PUNCH, NONE
   }

   private SecondaryControlOption activeSecondaryControlOption = SecondaryControlOption.KICK;

   private final HumanoidRobotKickMessenger kickMessenger;
   private final HumanoidRobotPunchMessenger punchMessenger;

   private final ROS2PublisherBasics<FootstepDataListMessage> footstepPublisher;
   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;
   private final ROS2PublisherBasics<REAStateRequestMessage> reaStateRequestPublisher;

   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;

   private final ConcurrentLinkedQueue<Runnable> queuedTasksToProcess = new ConcurrentLinkedQueue<>();

   public StepGeneratorJavaFXController(String robotName,
                                        JavaFXMessager messager,
                                        WalkingControllerParameters walkingControllerParameters,
                                        ROS2NodeInterface ros2Node,
                                        JavaFXRobotVisualizer javaFXRobotVisualizer,
                                        HumanoidRobotKickMessenger kickMessenger,
                                        HumanoidRobotPunchMessenger punchMessenger,
                                        RobotLowLevelMessenger lowLevelMessenger,
                                        SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons)
   {
      this.javaFXRobotVisualizer = javaFXRobotVisualizer;
      this.kickMessenger = kickMessenger;
      this.punchMessenger = punchMessenger;
      this.footPolygons = footPolygons;
      continuousStepController = new ContinuousStepController(walkingControllerParameters);

      messager.addTopicListener(SteppingParameters, continuousStepController::setJoystickStepParameters);
      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      ROS2Topic<?> controllerInputTopic = HumanoidControllerAPI.getInputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    controllerOutputTopic,
                                                    s -> continuousStepController.updateControllerMotionStatus(RobotMotionStatus.fromByte(s.takeNextData()
                                                                                                                                           .getRobotMotionStatus())));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    FootstepStatusMessage.class,
                                                    controllerOutputTopic,
                                                    s -> queuedTasksToProcess.add(() -> continuousStepController.consumeFootstepStatus(s.takeNextData())));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    PlanarRegionsListMessage.class,
                                                    REACommunicationProperties.outputTopic,
                                                    s -> queuedTasksToProcess.add(() -> continuousStepController.consumePlanarRegionsListMessage(s.takeNextData())));

      pauseWalkingPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(PauseWalkingMessage.class));
      footstepPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(FootstepDataListMessage.class));
      reaStateRequestPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(REAStateRequestMessage.class)
                                                                   .withTopic(REACommunicationProperties.inputTopic));

      continuousStepController.setFootstepMessenger(this::prepareFootsteps);
      continuousStepController.setPauseWalkingPublisher(this::sendPauseWalkingMessage);
      continuousStepController.setFootPoseProviders(robotSide ->
      {
         if (javaFXRobotVisualizer.isRobotConfigurationInitialized())
            return new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide));
         else
            return null;
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, CapturabilityBasedStatus.class, controllerOutputTopic, s ->
      {
         CapturabilityBasedStatus status = s.takeNextData();
         if (status == null)
            return;
         queuedTasksToProcess.add(() -> continuousStepController.setContactState(!status.getLeftFootSupportPolygon3d().isEmpty(),
                                                                                 !status.getRightFootSupportPolygon3d().isEmpty()));
      });

      trajectoryDuration = messager.createInput(WalkingTrajectoryDuration, 1.0);
      stepsAreAdjustable = messager.createInput(StepsAreAdjustable, false);

      setupKickAction(messager);
      setupPunchAction(messager);

      messager.addTopicListener(ButtonLeftBumperState, state ->
      {
         if (state == ButtonState.PRESSED)
            sendArmHomeConfiguration(RobotSide.values);
      });

      messager.addTopicListener(ButtonRightBumperState, state ->
      {
         if (state == ButtonState.PRESSED)
            continuousStepController.submitWalkingRequest(true);
         else if (state == ButtonState.RELEASED)
            continuousStepController.submitWalkingRequest(false);
      });

      messager.addFXTopicListener(LeftStickYAxis, continuousStepController::updateForwardVelocity);
      messager.addFXTopicListener(LeftStickXAxis, continuousStepController::updateLateralVelocity);
      messager.addFXTopicListener(RightStickXAxis, continuousStepController::updateTurningVelocity);
      messager.addTopicListener(DPadLeftState, state -> sendREAResumeRequest());
      messager.addTopicListener(DPadDownState, state -> sendREAClearRequest());

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    WalkingControllerFailureStatusMessage.class,
                                                    controllerOutputTopic,
                                                    s -> continuousStepController.submitWalkingRequest(false));
      messager.addTopicListener(ButtonSelectState, state -> continuousStepController.submitWalkingRequest(false));
      messager.addTopicListener(ButtonSelectState, state -> lowLevelMessenger.sendFreezeRequest());
      messager.addTopicListener(ButtonStartState, state -> continuousStepController.submitWalkingRequest(false));
      messager.addTopicListener(ButtonStartState, state -> lowLevelMessenger.sendStandRequest());
   }

   private YoVariableServer yoVariableServer = null;
   private final double yoVariableServerDT = 1.0 / 60.0;

   public void createYoVariableServer(DataServerSettings settings, LogModelProvider modelProvider)
   {
      if (yoVariableServer != null)
         return;

      String name = continuousStepController.getClass().getSimpleName();

      LogTools.info("{}: Trying to start YoVariableServer using port: {}.", name, settings.getPort());
      yoVariableServer = new YoVariableServer(getClass(), modelProvider, settings, yoVariableServerDT);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      continuousStepController.setupVisualization(yoGraphicsListRegistry);
      yoVariableServer.setMainRegistry(continuousStepController.getRegistry(), javaFXRobotVisualizer.getFullRobotModel().getElevator(), yoGraphicsListRegistry);
      yoVariableServer.start();
   }

   private long lastFootstepPublishTime = -1;
   private long lastYoVariableServerUpdateTime = -1;
   private long yoVariableServerStartTime = -1;

   public void update(long now)
   {
      try
      {
         while (!queuedTasksToProcess.isEmpty())
            queuedTasksToProcess.poll().run();

         continuousStepController.update();

         List<Node> footstepsToVisualize = footstepsToVisualizeReference.getAndSet(null);
         ObservableList<Node> children = rootNode.getChildren();

         if (!continuousStepController.isWalking())
         {
            children.clear();
         }
         else if (footstepsToVisualize != null)
         {
            children.clear();
            children.addAll(footstepsToVisualize);
         }

         if (yoVariableServer != null)
         {
            if (lastYoVariableServerUpdateTime == -1 || Conversions.nanosecondsToSeconds(now - lastYoVariableServerUpdateTime) >= yoVariableServerDT)
            {
               if (yoVariableServerStartTime == -1)
                  yoVariableServerStartTime = now;

               yoVariableServer.update(now - yoVariableServerStartTime);
               lastYoVariableServerUpdateTime = now;
            }
         }

         if (lastFootstepPublishTime == -1 || now - lastFootstepPublishTime >= footstepPublishingPeriod)
            publishFootstepRequest.set(true);

         if (publishFootstepRequest.getAndSet(false))
         {
            lastFootstepPublishTime = now;
            sendFootsteps();
         }
      }
      catch (Throwable e)
      {
         e.printStackTrace();
         LogTools.error("Caught exception, stopping animation timer.");
         stop();
      }
   }

   public void setActiveSecondaryControlOption(SecondaryControlOption activeSecondaryControlOption)
   {
      this.activeSecondaryControlOption = activeSecondaryControlOption;
   }

   private void setupPunchAction(JavaFXMessager messager)
   {
      messager.addTopicListener(ButtonXState, state -> processPunch(RobotSide.LEFT, state));
      messager.addTopicListener(ButtonYState, state -> processPunch(RobotSide.RIGHT, state));
   }

   private void setupKickAction(JavaFXMessager messager)
   {
      messager.addTopicListener(ButtonXState, state -> processToggleFlamingoMode(RobotSide.LEFT, state));
      messager.addTopicListener(ButtonBState, state -> processToggleFlamingoMode(RobotSide.RIGHT, state));
      messager.addTopicListener(ButtonYState, state -> processKick(state));
   }

   private void prepareFootsteps(FootstepDataListMessage footstepDataListMessage)
   {
      List<Node> footstepNode = new ArrayList<>();
      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);
         footstepDataMessage.setSwingHeight(continuousStepController.getJoystickStepParameters().getSwingHeight());
         footstepNode.add(createFootstep(footstepDataMessage));
      }
      footstepsToVisualizeReference.set(footstepNode);
      FootstepDataListMessage messageCopy = new FootstepDataListMessage(footstepDataListMessage);
      messageCopy.setAreFootstepsAdjustable(stepsAreAdjustable.get());
      footstepsToSendReference.set(messageCopy);
   }

   private void sendPauseWalkingMessage()
   {
      pauseWalkingPublisher.publish(HumanoidMessageTools.createPauseWalkingMessage(true));
   }

   private void sendFootsteps()
   {
      FootstepDataListMessage footstepsToSend = footstepsToSendReference.getAndSet(null);
      if (footstepsToSend != null && continuousStepController.isWalking())
      {
//         footstepsToSend.setAreFootstepsAdjustable(true);
         footstepPublisher.publish(footstepsToSend);
      }
   }

   private void sendREAClearRequest()
   {
      REAStateRequestMessage clearRequest = new REAStateRequestMessage();
      clearRequest.setRequestClear(true);
      reaStateRequestPublisher.publish(clearRequest);
   }

   private void sendREAResumeRequest()
   {
      REAStateRequestMessage resumeRequest = new REAStateRequestMessage();
      resumeRequest.setRequestResume(true);
      reaStateRequestPublisher.publish(resumeRequest);
   }

   private Node createFootstep(FootstepDataMessage footstepDataMessage)
   {
      RobotSide footSide = RobotSide.fromByte(footstepDataMessage.getRobotSide());
      return createFootstep(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation(), footColors.get(footSide), footPolygons.get(footSide));
   }

   private Node createFootstep(Point3DReadOnly position, QuaternionReadOnly orientation, Color footColor, ConvexPolygon2DReadOnly footPolygon)
   {
      MeshDataHolder polygon = MeshDataGenerator.ExtrudedPolygon(footPolygon, 0.025);
      polygon = MeshDataHolder.rotate(polygon, orientation);
      polygon = MeshDataHolder.translate(polygon, position);
      Mesh mesh = JavaFXMeshDataInterpreter.interpretMeshData(polygon, true);
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(footColor));
      return meshView;
   }

   private void processToggleFlamingoMode(RobotSide robotSide, ButtonState state)
   {
      if (activeSecondaryControlOption != SecondaryControlOption.KICK)
         return;
      if (state != ButtonState.PRESSED)
         return;
      if (continuousStepController.isInDoubleSupport())
         flamingoHomeStance(robotSide);
      else if (!continuousStepController.isFootInSupport(robotSide))
         putFootDown(robotSide);
   }

   private void processKick(ButtonState state)
   {
      if (activeSecondaryControlOption != SecondaryControlOption.KICK)
         return;
      if (continuousStepController.isInDoubleSupport())
         return;

      RobotSide kickSide = continuousStepController.isFootInSupport(RobotSide.RIGHT) ? RobotSide.LEFT : RobotSide.RIGHT;

      if (state == ButtonState.PRESSED)
         kick(kickSide);
      else if (state == ButtonState.RELEASED)
         flamingoHomeStance(kickSide);
   }

   private void processPunch(RobotSide robotSide, ButtonState state)
   {
      if (activeSecondaryControlOption != SecondaryControlOption.PUNCH)
         return;
      if (state == ButtonState.PRESSED)
         sendArmStraightConfiguration(robotSide);
      else
         sendArmHomeConfiguration(robotSide);
   }

   private void sendArmHomeConfiguration(RobotSide... robotSides)
   {
      punchMessenger.sendArmHomeConfiguration(trajectoryDuration.get(), robotSides);
   }

   private void sendArmStraightConfiguration(RobotSide robotSide)
   {
      punchMessenger.sendArmStraightConfiguration(trajectoryDuration.get(), robotSide);
   }

   private void flamingoHomeStance(RobotSide robotSide)
   {
      kickMessenger.sendFlamingoHomeStance(robotSide,
                                           trajectoryDuration.get(),
                                           continuousStepController.getJoystickStepParameters().getDefaultStepWidth(),
                                           javaFXRobotVisualizer.getFullRobotModel().getSoleFrames());
   }

   private void putFootDown(RobotSide robotSide)
   {
      if (continuousStepController.isFootInSupport(robotSide))
         return;
      kickMessenger.sendPutFootDown(robotSide,
                                    trajectoryDuration.get(),
                                    continuousStepController.getJoystickStepParameters().getDefaultStepWidth(),
                                    javaFXRobotVisualizer.getFullRobotModel().getSoleFrames());
   }

   private void kick(RobotSide robotSide)
   {
      if (continuousStepController.isFootInSupport(robotSide))
         return;
      kickMessenger.sendKick(robotSide,
                             trajectoryDuration.get(),
                             continuousStepController.getJoystickStepParameters().getDefaultStepWidth(),
                             javaFXRobotVisualizer.getFullRobotModel().getSoleFrames());
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
      sendPauseWalkingMessage();
      if (yoVariableServer != null)
      {
         yoVariableServer.close();
         yoVariableServer = null;
      }
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
