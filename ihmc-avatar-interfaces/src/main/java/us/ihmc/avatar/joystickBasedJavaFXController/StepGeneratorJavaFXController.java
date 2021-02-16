package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.SteppingParameters;
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
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import javafx.animation.AnimationTimer;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class StepGeneratorJavaFXController
{
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory("FootstepPublisher"));
   private final SideDependentList<Color> footColors = new SideDependentList<>(Color.CRIMSON, Color.YELLOWGREEN);

   private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
   private final DoubleProperty turningVelocityProperty = new SimpleDoubleProperty(this, "turningVelocityProperty", 0.0);
   private final DoubleProperty forwardVelocityProperty = new SimpleDoubleProperty(this, "forwardVelocityProperty", 0.0);
   private final DoubleProperty lateralVelocityProperty = new SimpleDoubleProperty(this, "lateralVelocityProperty", 0.0);

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

   private final AtomicReference<JoystickStepParameters> stepParametersReference;
   private final AtomicReference<Double> trajectoryDuration;
   private final AtomicBoolean isWalking = new AtomicBoolean(false);
   private final AtomicBoolean hasSuccessfullyStoppedWalking = new AtomicBoolean(false);
   private final JavaFXRobotVisualizer javaFXRobotVisualizer;

   private final AtomicBoolean isLeftFootInSupport = new AtomicBoolean(false);
   private final AtomicBoolean isRightFootInSupport = new AtomicBoolean(false);
   private final SideDependentList<AtomicBoolean> isFootInSupport = new SideDependentList<>(isLeftFootInSupport, isRightFootInSupport);
   private final BooleanProvider isInDoubleSupport = () -> isLeftFootInSupport.get() && isRightFootInSupport.get();
   private final DoubleProvider stepTime;
   private final BoundingBoxCollisionDetector collisionDetector;
   private final SteppingParameters steppingParameters;
   private final SnapAndWiggleSingleStepParameters snapAndWiggleParameters = new SnapAndWiggleSingleStepParameters();

   public enum SecondaryControlOption
   {
      KICK, PUNCH, NONE
   }

   private SecondaryControlOption activeSecondaryControlOption = SecondaryControlOption.KICK;

   private final HumanoidRobotKickMessenger kickMessenger;
   private final HumanoidRobotPunchMessenger punchMessenger;

   private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;
   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
   private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
   private final ConvexPolygon2D footPolygonToWiggle = new ConvexPolygon2D();

   private boolean supportFootPosesInitialized = false;
   private final SideDependentList<FramePose3D> lastSupportFootPoses = new SideDependentList<>(null, null);
   private final ConcurrentLinkedQueue<Runnable> queuedTasksToProcess = new ConcurrentLinkedQueue<>();

   private final AtomicReference<Boolean> walkingRequest = new AtomicReference<>(null);

   public StepGeneratorJavaFXController(String robotName, JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters, ROS2Node ros2Node,
                                        JavaFXRobotVisualizer javaFXRobotVisualizer, HumanoidRobotKickMessenger kickMessenger,
                                        HumanoidRobotPunchMessenger punchMessenger, RobotLowLevelMessenger lowLevelMessenger,
                                        SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons)
   {
      this.javaFXRobotVisualizer = javaFXRobotVisualizer;
      this.kickMessenger = kickMessenger;
      this.punchMessenger = punchMessenger;
      this.footPolygons = footPolygons;
      continuousStepGenerator.setNumberOfFootstepsToPlan(10);
      continuousStepGenerator.setDesiredTurningVelocityProvider(() -> turningVelocityProperty.get());
      continuousStepGenerator.setDesiredVelocityProvider(() -> new Vector2D(forwardVelocityProperty.get(), lateralVelocityProperty.get()));
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.setFootstepAdjustment(this::adjustFootstep);
      continuousStepGenerator.setFootstepMessenger(this::prepareFootsteps);
      continuousStepGenerator.setFootPoseProvider(robotSide -> lastSupportFootPoses.get(robotSide));
      continuousStepGenerator.addFootstepValidityIndicator(this::isStepSnappable);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeDistanceFromObstacle);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeStepHeight);

      snapAndWiggleParameters.setFootLength(walkingControllerParameters.getSteppingParameters().getFootLength());
      snapAndWiggleSingleStep = new SnapAndWiggleSingleStep(snapAndWiggleParameters);

      steppingParameters = walkingControllerParameters.getSteppingParameters();

      stepParametersReference = messager.createInput(SteppingParameters, new JoystickStepParameters(walkingControllerParameters));
      ROS2Topic<?> controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Topic<?> controllerInputTopic = ROS2Tools.getControllerInputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerOutputTopic, s ->
      {
         RobotMotionStatus newStatus = RobotMotionStatus.fromByte(s.takeNextData().getRobotMotionStatus());
         // We only want to verify that the last PauseWalking sent has been successfully executed once.
         // Considering that the user may use a separate app to get the robot to walk, we do not want to interfere with the other app.
         if (hasSuccessfullyStoppedWalking.get() || isWalking.get())
            return;
         if (newStatus == null)
            return;
         if (newStatus == RobotMotionStatus.STANDING)
            hasSuccessfullyStoppedWalking.set(true);
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepStatusMessage.class, controllerOutputTopic, s ->
      {
         FootstepStatusMessage footstepStatus = s.takeNextData();
         queuedTasksToProcess.add(() ->
         {
            continuousStepGenerator.consumeFootstepStatus(footstepStatus);

            if (footstepStatus.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
            {
               lastSupportFootPoses.put(RobotSide.fromByte(footstepStatus.getRobotSide()),
                                        new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                        footstepStatus.getActualFootPositionInWorld(),
                                                        footstepStatus.getActualFootOrientationInWorld()));
            }
         });
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    PlanarRegionsListMessage.class,
                                                    REACommunicationProperties.outputTopic,
                                                    s -> planarRegionsListMessage.set(s.takeNextData()));

      pauseWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PauseWalkingMessage.class, controllerInputTopic);
      footstepPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootstepDataListMessage.class, controllerInputTopic);
      reaStateRequestPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, REAStateRequestMessage.class, REACommunicationProperties.inputTopic);

      trajectoryDuration = messager.createInput(WalkingTrajectoryDuration, 1.0);
      stepTime = () -> stepParametersReference.get().getSwingDuration() + stepParametersReference.get().getTransferDuration();

      setupKickAction(messager);
      setupPunchAction(messager);

      messager.registerTopicListener(ButtonLeftBumperState, state ->
      {
         if (state == ButtonState.PRESSED)
            sendArmHomeConfiguration(RobotSide.values);
      });

      messager.registerTopicListener(ButtonRightBumperState, state ->
      {
         if (state == ButtonState.PRESSED)
            walkingRequest.set(true);
         else if (state == ButtonState.RELEASED)
            walkingRequest.set(false);
      });

      messager.registerJavaFXSyncedTopicListener(LeftStickYAxis, this::updateForwardVelocity);
      messager.registerJavaFXSyncedTopicListener(LeftStickXAxis, this::updateLateralVelocity);
      messager.registerJavaFXSyncedTopicListener(RightStickXAxis, this::updateTurningVelocity);
      messager.registerTopicListener(DPadLeftState, state -> sendREAResumeRequest());
      messager.registerTopicListener(DPadDownState, state -> sendREAClearRequest());

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    WalkingControllerFailureStatusMessage.class,
                                                    controllerOutputTopic,
                                                    s -> walkingRequest.set(false));
      messager.registerTopicListener(ButtonSelectState, state -> walkingRequest.set(false));
      messager.registerTopicListener(ButtonSelectState, state -> lowLevelMessenger.sendFreezeRequest());
      messager.registerTopicListener(ButtonStartState, state -> walkingRequest.set(false));
      messager.registerTopicListener(ButtonStartState, state -> lowLevelMessenger.sendStandRequest());
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, CapturabilityBasedStatus.class, controllerOutputTopic, s ->
      {
         CapturabilityBasedStatus status = s.takeNextData();
         queuedTasksToProcess.add(() ->
         {
            isLeftFootInSupport.set(!status.getLeftFootSupportPolygon3d().isEmpty());
            isRightFootInSupport.set(!status.getRightFootSupportPolygon3d().isEmpty());
         });
      });

      double collisionBoxDepth = 0.65;
      double collisionBoxWidth = 1.15;
      double collisionBoxHeight = 1.0;
      double collisionXYProximityCheck = 0.01;
      collisionDetector = new BoundingBoxCollisionDetector();
      collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight, collisionXYProximityCheck);
   }

   public void update(long now)
   {
      try
      {
         while (!queuedTasksToProcess.isEmpty())
            queuedTasksToProcess.poll().run();

         if (!supportFootPosesInitialized && javaFXRobotVisualizer.isRobotConfigurationInitialized())
         {
            if (isLeftFootInSupport.get() && isRightFootInSupport.get())
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  lastSupportFootPoses.put(robotSide, new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide)));
               }

               supportFootPosesInitialized = true;
            }
            else
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  if (!isFootInSupport.get(robotSide).get())
                     LogTools.warn(robotSide.getPascalCaseName() + " foot is not in support, cannot initialize foot poses.");
               }
            }
         }

         if (!supportFootPosesInitialized)
            return;

         for (RobotSide robotSide : RobotSide.values)
         {
            if (isFootInSupport.get(robotSide).get())
            { // Touchdown may not have been made with the foot properly settled, so we update the support foot pose if its current pose is lower.
               MovingReferenceFrame soleFrame = javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide);
               double currentHeight = soleFrame.getTransformToWorldFrame().getTranslationZ();
               if (currentHeight < lastSupportFootPoses.get(robotSide).getZ())
                  lastSupportFootPoses.put(robotSide, new FramePose3D(soleFrame));
            }
         }

         PlanarRegionsListMessage latestMessage = planarRegionsListMessage.getAndSet(null);

         if (latestMessage != null)
         {
            PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(latestMessage);
            snapAndWiggleSingleStep.setPlanarRegions(planarRegionsList);
            collisionDetector.setPlanarRegionsList(new PlanarRegionsList(planarRegionsList.getPlanarRegionsAsList().stream()
                                                                                          .filter(region -> region.getConvexHull()
                                                                                                                  .getArea() >= snapAndWiggleParameters.getMinPlanarRegionArea())
                                                                                          .collect(Collectors.toList())));
            this.planarRegionsList.set(planarRegionsList);
         }

         Boolean newWalkingRequest = walkingRequest.getAndSet(null);

         if (newWalkingRequest != null)
         {
            if (newWalkingRequest)
               startWalking(true);
            else
               stopWalking(true);
         }

         JoystickStepParameters stepParameters = stepParametersReference.get();
         continuousStepGenerator.setFootstepTiming(stepParameters.getSwingDuration(), stepParameters.getTransferDuration());
         continuousStepGenerator.update(Double.NaN);

         List<Node> footstepsToVisualize = footstepsToVisualizeReference.getAndSet(null);
         ObservableList<Node> children = rootNode.getChildren();

         if (!continuousStepGenerator.isWalking())
         {
            children.clear();
         }
         else if (footstepsToVisualize != null)
         {
            children.clear();
            children.addAll(footstepsToVisualize);
         }
      }
      catch (Throwable e)
      {
         e.printStackTrace();
         LogTools.error("Caught exception, stopping animation timer.");
         stop();
      }
   }

   private FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide)
   {
      FramePose3D adjustedBasedOnStanceFoot = new FramePose3D();
      adjustedBasedOnStanceFoot.getPosition().set(footstepPose.getPosition());
      adjustedBasedOnStanceFoot.setZ(continuousStepGenerator.getCurrentSupportFootPose().getZ());
      adjustedBasedOnStanceFoot.getOrientation().set(footstepPose.getOrientation());

      if (planarRegionsList.get() != null)
      {
         FramePose3D wiggledPose = new FramePose3D(adjustedBasedOnStanceFoot);
         footPolygonToWiggle.set(footPolygons.get(footSide));
         try
         {
            snapAndWiggleSingleStep.snapAndWiggle(wiggledPose, footPolygonToWiggle, forwardVelocityProperty.get() > 0.0);
            if (wiggledPose.containsNaN())
               return adjustedBasedOnStanceFoot;
         }
         catch (SnappingFailedException e)
         {
            /*
             * It's fine if the snap & wiggle fails, can be because there no planar regions around the footstep.
             * Let's just keep the adjusted footstep based on the pose of the current stance foot.
             */
         }
         return wiggledPose;
      }
      else
      {
         return adjustedBasedOnStanceFoot;
      }
   }

   public void setActiveSecondaryControlOption(SecondaryControlOption activeSecondaryControlOption)
   {
      this.activeSecondaryControlOption = activeSecondaryControlOption;
   }

   private void setupPunchAction(JavaFXMessager messager)
   {
      messager.registerTopicListener(ButtonXState, state -> processPunch(RobotSide.LEFT, state));
      messager.registerTopicListener(ButtonYState, state -> processPunch(RobotSide.RIGHT, state));
   }

   private void setupKickAction(JavaFXMessager messager)
   {
      messager.registerTopicListener(ButtonXState, state -> processToggleFlamingoMode(RobotSide.LEFT, state));
      messager.registerTopicListener(ButtonBState, state -> processToggleFlamingoMode(RobotSide.RIGHT, state));
      messager.registerTopicListener(ButtonYState, state -> processKick(state));
   }

   private void updateForwardVelocity(double alpha)
   {
      double minMaxVelocity = stepParametersReference.get().getMaxStepLength() / stepTime.getValue();
      forwardVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void updateLateralVelocity(double alpha)
   {
      double minMaxVelocity = stepParametersReference.get().getMaxStepWidth() / stepTime.getValue();
      lateralVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void updateTurningVelocity(double alpha)
   {
      double minMaxVelocity = (stepParametersReference.get().getTurnMaxAngleOutward() - stepParametersReference.get().getTurnMaxAngleInward())
            / stepTime.getValue();
      if (forwardVelocityProperty.get() < -1.0e-10)
         alpha = -alpha;
      turningVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void startWalking(boolean confirm)
   {
      if (confirm)
      {
         isWalking.set(true);
         continuousStepGenerator.startWalking();
         hasSuccessfullyStoppedWalking.set(false);
      }
   }

   private void stopWalking(boolean confirm)
   {
      if (confirm)
      {
         isWalking.set(false);
         footstepsToSendReference.set(null);
         continuousStepGenerator.stopWalking();
         sendPauseMessage();
      }
   }

   private void sendPauseMessage()
   {
      PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
      pauseWalkingMessage.setPause(true);
      pauseWalkingPublisher.publish(pauseWalkingMessage);
   }

   private void prepareFootsteps(FootstepDataListMessage footstepDataListMessage)
   {
      List<Node> footstepNode = new ArrayList<>();
      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);
         footstepDataMessage.setSwingHeight(stepParametersReference.get().getSwingHeight());
         footstepNode.add(createFootstep(footstepDataMessage));
      }
      footstepsToVisualizeReference.set(footstepNode);
      //      footstepDataListMessage.setAreFootstepsAdjustable(true);
      footstepsToSendReference.set(new FootstepDataListMessage(footstepDataListMessage));
   }

   private void sendFootsteps()
   {
      FootstepDataListMessage footstepsToSend = footstepsToSendReference.getAndSet(null);
      if (footstepsToSend != null && isWalking.get())
      {
         footstepPublisher.publish(footstepsToSend);
      }

      if (!isWalking.get())
      {
         // Only send pause request if we think the command has not been executed yet. This is to be more robust in case packets are dropped.
         if (!hasSuccessfullyStoppedWalking.get())
            sendPauseMessage();
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

   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (planarRegionsList.get() == null)
         return true;

      double halfStanceWidth = 0.5 * steppingParameters.getInPlaceWidth();

      /** Shift box vertically by max step up, regions below this could be steppable */
      double heightOffset = steppingParameters.getMaxStepUp();

      double soleYaw = touchdownPose.getYaw();
      double lateralOffset = swingSide.negateIfLeftSide(halfStanceWidth);
      double offsetX = -lateralOffset * Math.sin(soleYaw);
      double offsetY = lateralOffset * Math.cos(soleYaw);
      collisionDetector.setBoxPose(touchdownPose.getX() + offsetX, touchdownPose.getY() + offsetY, touchdownPose.getZ() + heightOffset, soleYaw);

      return !collisionDetector.checkForCollision().isCollisionDetected();
   }

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final PlanarRegion tempRegion = new PlanarRegion();

   private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (planarRegionsList.get() == null)
         return true;

      tempTransform.getTranslation().set(touchdownPose.getPosition().getX(), touchdownPose.getPosition().getY(), 0.0);
      tempTransform.getRotation().setToYawOrientation(touchdownPose.getYaw());

      footPolygon.set(footPolygons.get(swingSide));
      footPolygon.applyTransform(tempTransform, false);

      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();

      return PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, Double.POSITIVE_INFINITY, tempRegion) != null;
   }

   private void processToggleFlamingoMode(RobotSide robotSide, ButtonState state)
   {
      if (activeSecondaryControlOption != SecondaryControlOption.KICK)
         return;
      if (state != ButtonState.PRESSED)
         return;
      if (isInDoubleSupport.getValue())
         flamingoHomeStance(robotSide);
      else if (!isFootInSupport.get(robotSide).get())
         putFootDown(robotSide);
   }

   private void processKick(ButtonState state)
   {
      if (activeSecondaryControlOption != SecondaryControlOption.KICK)
         return;
      if (isInDoubleSupport.getValue())
         return;

      RobotSide kickSide = isRightFootInSupport.get() ? RobotSide.LEFT : RobotSide.RIGHT;

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
                                           stepParametersReference.get().getDefaultStepWidth(),
                                           javaFXRobotVisualizer.getFullRobotModel().getSoleFrames());
   }

   private void putFootDown(RobotSide robotSide)
   {
      if (isFootInSupport.get(robotSide).get())
         return;
      kickMessenger.sendPutFootDown(robotSide,
                                    trajectoryDuration.get(),
                                    stepParametersReference.get().getDefaultStepWidth(),
                                    javaFXRobotVisualizer.getFullRobotModel().getSoleFrames());
   }

   private void kick(RobotSide robotSide)
   {
      if (isFootInSupport.get(robotSide).get())
         return;
      kickMessenger.sendKick(robotSide,
                             trajectoryDuration.get(),
                             stepParametersReference.get().getDefaultStepWidth(),
                             javaFXRobotVisualizer.getFullRobotModel().getSoleFrames());
   }

   public void start()
   {
      animationTimer.start();
      executorService.scheduleAtFixedRate(this::sendFootsteps, 0, 500, TimeUnit.MILLISECONDS);
   }

   public void stop()
   {
      animationTimer.stop();
      executorService.shutdownNow();
      PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
      pauseWalkingMessage.setPause(true);
      pauseWalkingPublisher.publish(pauseWalkingMessage);
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
