package us.ihmc.atlas.joystickBasedStepping;

import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonAState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonSelectState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonStartState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.LeftStickXAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.LeftStickYAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.RightStickXAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingSwingDuration;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingSwingHeight;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingTransferDuration;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
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
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class StepGeneratorJavaFXController
{
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("FootstepPublisher"));
   private final SideDependentList<Color> footColors = new SideDependentList<Color>(Color.CRIMSON, Color.YELLOWGREEN);

   private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
   private final DoubleProperty headingVelocityProperty = new SimpleDoubleProperty(this, "headingVelocityProperty", 0.0);
   private final DoubleProperty forwardVelocityProperty = new SimpleDoubleProperty(this, "forwardVelocityProperty", 0.0);
   private final DoubleProperty lateralVelocityProperty = new SimpleDoubleProperty(this, "lateralVelocityProperty", 0.0);

   private final AnimationTimer animationTimer;
   private final PacketCommunicator packetCommunicator;
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final AtomicReference<List<Node>> footstepsToVisualizeReference = new AtomicReference<>(null);

   private final Group rootNode = new Group();

   private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);

   private final AtomicReference<Double> swingHeight;
   private final AtomicReference<Double> swingDuration;
   private final AtomicReference<Double> transferDuration;

   public StepGeneratorJavaFXController(JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters,
                                        PacketCommunicator packetCommunicator, JavaFXRobotVisualizer javaFXRobotVisualizer)
   {
      this.packetCommunicator = packetCommunicator;
      continuousStepGenerator.setNumberOfFootstepsToPlan(10);
      continuousStepGenerator.setDesiredHeadingProvider(() -> headingVelocityProperty.get());
      continuousStepGenerator.setDesiredVelocityProvider(() -> new Vector2D(forwardVelocityProperty.get(), lateralVelocityProperty.get()));
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.setSupportFootBasedFootstepAdjustment();
      continuousStepGenerator.setFootstepMessenger(this::prepareFootsteps);
      continuousStepGenerator.setFootPoseProvider(robotSide -> new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide)));
      packetCommunicator.attachListener(FootstepStatusMessage.class, continuousStepGenerator::consumeFootstepStatus);

      swingHeight = messager.createInput(WalkingSwingHeight, 0.12);
      swingDuration = messager.createInput(WalkingSwingDuration, walkingControllerParameters.getDefaultSwingTime());
      transferDuration = messager.createInput(WalkingTransferDuration, walkingControllerParameters.getDefaultTransferTime());

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            continuousStepGenerator.setFootstepTiming(swingDuration.get(), transferDuration.get());
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
      };

      double footLength = walkingControllerParameters.getSteppingParameters().getFootLength();
      double toeWidth = walkingControllerParameters.getSteppingParameters().getToeWidth();
      double footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      footPolygon.addVertex(footLength / 2.0, toeWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -toeWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.update();

      messager.registerTopicListener(ButtonAState, state -> toggleWalking(state == ButtonState.PRESSED));
      messager.registerJavaFXSyncedTopicListener(LeftStickYAxis, this::updateForwardVelocity);
      messager.registerJavaFXSyncedTopicListener(LeftStickXAxis, this::updateLateralVelocity);
      messager.registerJavaFXSyncedTopicListener(RightStickXAxis, this::updateHeadingVelocity);

      packetCommunicator.attachListener(WalkingControllerFailureStatusMessage.class, packet -> stopWalking(true));
      messager.registerTopicListener(ButtonSelectState, state -> sendLowLevelControlModeRequest(AtlasLowLevelControlMode.FREEZE));
      messager.registerTopicListener(ButtonStartState, state -> sendLowLevelControlModeRequest(AtlasLowLevelControlMode.STAND_PREP));
   }

   private void updateForwardVelocity(double alpha)
   {
      double minMaxVelocity = 0.15;
      forwardVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void updateLateralVelocity(double alpha)
   {
      double minMaxVelocity = 0.15;
      lateralVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void updateHeadingVelocity(double alpha)
   {
      double minMaxVelocity = Math.PI / 6.0;
      headingVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void toggleWalking(boolean confirm)
   {
      if (confirm)
      {
         continuousStepGenerator.toggleWalking();
         if (!continuousStepGenerator.isWalking())
         {
            PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
            pauseWalkingMessage.setPause(true);
            packetCommunicator.send(pauseWalkingMessage);
         }
      }
   }

   private void stopWalking(boolean confirm)
   {
      if (confirm)
      {
         continuousStepGenerator.stopWalking();
         PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
         pauseWalkingMessage.setPause(true);
         packetCommunicator.send(pauseWalkingMessage);
      }
   }

   public void sendLowLevelControlModeRequest(AtlasLowLevelControlMode mode)
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.setRequestedAtlasLowLevelControlMode(mode.toByte());
      packetCommunicator.send(message);
   }

   private void prepareFootsteps(FootstepDataListMessage footstepDataListMessage)
   {
      List<Node> footstepNode = new ArrayList<>();
      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);
         footstepDataMessage.setSwingHeight(swingHeight.get());
         footstepNode.add(createFootstep(footstepDataMessage));
      }
      footstepsToVisualizeReference.set(footstepNode);
      footstepsToSendReference.set(new FootstepDataListMessage(footstepDataListMessage));
   }

   private void sendFootsteps()
   {
      FootstepDataListMessage footstepsToSend = footstepsToSendReference.getAndSet(null);
      if (footstepsToSend != null)
      {
         packetCommunicator.send(footstepsToSend);
      }
   }

   private Node createFootstep(FootstepDataMessage footstepDataMessage)
   {
      return createFootstep(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation(),
                            footColors.get(RobotSide.fromByte(footstepDataMessage.getRobotSide())));
   }

   private Node createFootstep(Point3DReadOnly position, QuaternionReadOnly orientation, Color footColor)
   {
      MeshDataHolder polygon = MeshDataGenerator.ExtrudedPolygon(footPolygon, 0.025);
      polygon = MeshDataHolder.rotate(polygon, orientation);
      polygon = MeshDataHolder.translate(polygon, position);
      Mesh mesh = JavaFXMeshDataInterpreter.interpretMeshData(polygon, true);
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(footColor));
      return meshView;
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
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
