package us.ihmc.avatar.joystickBasedJavaFXController;

import controller_msgs.msg.dds.*;
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
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.SteppingParameters;
import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.WalkingTrajectoryDuration;
import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.*;

public class StepGeneratorJavaFXController
{
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("FootstepPublisher"));
   private final SideDependentList<Color> footColors = new SideDependentList<Color>(Color.CRIMSON, Color.YELLOWGREEN);

   private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
   private final DoubleProperty turningVelocityProperty = new SimpleDoubleProperty(this, "turningVelocityProperty", 0.0);
   private final DoubleProperty forwardVelocityProperty = new SimpleDoubleProperty(this, "forwardVelocityProperty", 0.0);
   private final DoubleProperty lateralVelocityProperty = new SimpleDoubleProperty(this, "lateralVelocityProperty", 0.0);

   private final AnimationTimer animationTimer;
   private final AtomicReference<List<Node>> footstepsToVisualizeReference = new AtomicReference<>(null);

   private final Group rootNode = new Group();

   private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);

   private final AtomicReference<JoystickStepParameters> stepParametersReference;
   private final AtomicReference<Double> trajectoryDuration;
   private final AtomicBoolean isWalking = new AtomicBoolean(false);
   private final JavaFXRobotVisualizer javaFXRobotVisualizer;

   private final AtomicBoolean isLeftFootInSupport = new AtomicBoolean(false);
   private final AtomicBoolean isRightFootInSupport = new AtomicBoolean(false);
   private final SideDependentList<AtomicBoolean> isFootInSupport = new SideDependentList<>(isLeftFootInSupport, isRightFootInSupport);
   private final BooleanProvider isInDoubleSupport = () -> isLeftFootInSupport.get() && isRightFootInSupport.get();
   private final DoubleProvider stepTime;

   public enum SecondaryControlOption
   {
      KICK, PUNCH
   };

   private SecondaryControlOption activeSecondaryControlOption = SecondaryControlOption.KICK;

   private final HumanoidRobotKickMessenger kickMessenger;
   private final HumanoidRobotPunchMessenger punchMessenger;

   private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;

   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
   private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
   private final ConvexPolygon2D footPolygonToWiggle = new ConvexPolygon2D();

   public StepGeneratorJavaFXController(String robotName, JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters, Ros2Node ros2Node,
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
      continuousStepGenerator.setFootPoseProvider(robotSide -> new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide)));
      continuousStepGenerator.addFootstepValidityIndicator(this::isStepSnappable);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeDistanceFromObstacle);

      SnapAndWiggleSingleStepParameters parameters = new SnapAndWiggleSingleStepParameters();
      parameters.setFootLength(walkingControllerParameters.getSteppingParameters().getFootLength());
      snapAndWiggleSingleStep = new SnapAndWiggleSingleStep(parameters);

      stepParametersReference = messager.createInput(SteppingParameters, new JoystickStepParameters(walkingControllerParameters));
      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.MessageTopicNameGenerator controllerSubGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           FootstepStatusMessage.class,
                                           controllerPubGenerator,
                                           s -> continuousStepGenerator.consumeFootstepStatus(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           PlanarRegionsListMessage.class,
                                           REACommunicationProperties.publisherTopicNameGenerator, s -> planarRegionsListMessage.set(s.takeNextData()));

      pauseWalkingPublisher = ROS2Tools.createPublisher(ros2Node, PauseWalkingMessage.class, controllerSubGenerator);
      footstepPublisher = ROS2Tools.createPublisher(ros2Node, FootstepDataListMessage.class, controllerSubGenerator);

      trajectoryDuration = messager.createInput(WalkingTrajectoryDuration, 1.0);
      stepTime = () -> stepParametersReference.get().getSwingDuration() + stepParametersReference.get().getTransferDuration();

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            try
            {
               PlanarRegionsListMessage latestMessage = planarRegionsListMessage.getAndSet(null);
               if(latestMessage != null)
               {
                  PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(latestMessage);
                  snapAndWiggleSingleStep.setPlanarRegions(planarRegionsList);
                  StepGeneratorJavaFXController.this.planarRegionsList.set(planarRegionsList);
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
      };

      setupKickAction(messager);
      setupPunchAction(messager);

      messager.registerTopicListener(ButtonLeftBumperState, state ->
      {
         if (state == ButtonState.PRESSED)
            sendArmHomeConfiguration(RobotSide.values);
      });
      messager.registerTopicListener(ButtonRightBumperState, state -> startWalking(state == ButtonState.PRESSED));
      messager.registerTopicListener(ButtonRightBumperState, state -> stopWalking(state == ButtonState.RELEASED));
      messager.registerJavaFXSyncedTopicListener(LeftStickYAxis, this::updateForwardVelocity);
      messager.registerJavaFXSyncedTopicListener(LeftStickXAxis, this::updateLateralVelocity);
      messager.registerJavaFXSyncedTopicListener(RightStickXAxis, this::updateTurningVelocity);

      ROS2Tools.createCallbackSubscription(ros2Node, WalkingControllerFailureStatusMessage.class, controllerPubGenerator, s -> stopWalking(true));
      messager.registerTopicListener(ButtonSelectState, state -> stopWalking(true));
      messager.registerTopicListener(ButtonSelectState, state -> lowLevelMessenger.sendFreezeRequest());
      messager.registerTopicListener(ButtonStartState, state -> stopWalking(true));
      messager.registerTopicListener(ButtonStartState, state -> lowLevelMessenger.sendStandRequest());
      ROS2Tools.createCallbackSubscription(ros2Node, CapturabilityBasedStatus.class, controllerPubGenerator, s ->
      {
         CapturabilityBasedStatus status = s.takeNextData();
         isLeftFootInSupport.set(!status.getLeftFootSupportPolygon2d().isEmpty());
         isRightFootInSupport.set(!status.getRightFootSupportPolygon2d().isEmpty());
      });
   }

   private FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide)
   {
      FramePose3D adjustedBasedOnStanceFoot = new FramePose3D();
      adjustedBasedOnStanceFoot.getPosition().set(footstepPose.getPosition());
      adjustedBasedOnStanceFoot.setZ(continuousStepGenerator.getCurrentSupportFootPose().getZ());
      adjustedBasedOnStanceFoot.setOrientation(footstepPose.getOrientation());

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
             * It's fine if the snap & wiggle fails, can be because there no
             * planar regions around the footstep. Let's just keep the adjusted
             * footstep based on the pose of the current stance foot.
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
         sendPauseMessage();
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

   private final Point3D bodyCenter = new Point3D();

   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly solePose, RobotSide robotSide)
   {
      double groundOffset = 0.2;
      double bodyRadius = 0.5;

      bodyCenter.set(solePose.getPosition());
      bodyCenter.addZ(bodyRadius + groundOffset);

      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         Point3D closestPoint = PlanarRegionTools.closestPointOnPlane(bodyCenter, planarRegionsList.getPlanarRegion(i));
         if(closestPoint == null)
            continue;

         double distance = closestPoint.distance(bodyCenter);
         if(distance < bodyRadius)
         {
            return false;
         }
      }

      return true;
   }

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final PlanarRegion tempRegion = new PlanarRegion();

   private boolean isStepSnappable(FramePose3DReadOnly solePose, RobotSide robotSide)
   {
      tempTransform.setTranslation(solePose.getPosition().getX(), solePose.getPosition().getY(), 0.0);
      tempTransform.setRotationYaw(solePose.getYaw());

      footPolygon.set(footPolygons.get(robotSide));
      footPolygon.applyTransform(tempTransform, false);

      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
      return PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, tempRegion) != null;
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
      executorService.scheduleAtFixedRate(this::sendFootsteps, 0, 100, TimeUnit.MILLISECONDS);
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
