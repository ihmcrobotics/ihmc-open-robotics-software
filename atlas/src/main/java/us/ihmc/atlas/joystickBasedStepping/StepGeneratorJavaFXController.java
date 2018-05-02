package us.ihmc.atlas.joystickBasedStepping;

import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingSwingDuration;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingSwingHeight;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingTrajectoryDuration;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingTransferDuration;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.ButtonBState;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.ButtonLeftBumperState;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.ButtonRightBumperState;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.ButtonSelectState;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.ButtonStartState;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.ButtonXState;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.ButtonYState;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.LeftStickXAxis;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.LeftStickYAxis;
import static us.ihmc.atlas.joystickBasedStepping.XBoxOneJavaFXController.RightStickXAxis;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createTrajectoryPoint1DMessage;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.BooleanProvider;

public class StepGeneratorJavaFXController
{
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("FootstepPublisher"));
   private final SideDependentList<Color> footColors = new SideDependentList<Color>(Color.CRIMSON, Color.YELLOWGREEN);

   private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
   private final DoubleProperty turningVelocityProperty = new SimpleDoubleProperty(this, "turningVelocityProperty", 0.0);
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
   private final AtomicReference<Double> trajectoryDuration;
   private final AtomicBoolean isWalking = new AtomicBoolean(false);
   private final JavaFXRobotVisualizer javaFXRobotVisualizer;
   private final double inPlaceStepWidth;

   private final AtomicBoolean isLeftFootInSupport = new AtomicBoolean(false);
   private final AtomicBoolean isRightFootInSupport = new AtomicBoolean(false);
   private final SideDependentList<AtomicBoolean> isFootInSupport = new SideDependentList<AtomicBoolean>(isLeftFootInSupport, isRightFootInSupport);
   private final BooleanProvider isInDoubleSupport = () -> isLeftFootInSupport.get() && isRightFootInSupport.get();

   public StepGeneratorJavaFXController(JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters, PacketCommunicator packetCommunicator,
                                        JavaFXRobotVisualizer javaFXRobotVisualizer)
   {
      this.packetCommunicator = packetCommunicator;
      this.javaFXRobotVisualizer = javaFXRobotVisualizer;
      continuousStepGenerator.setNumberOfFootstepsToPlan(10);
      continuousStepGenerator.setDesiredTurningVelocityProvider(() -> turningVelocityProperty.get());
      continuousStepGenerator.setDesiredVelocityProvider(() -> new Vector2D(forwardVelocityProperty.get(), lateralVelocityProperty.get()));
      continuousStepGenerator.configureWith(walkingControllerParameters);
      inPlaceStepWidth = walkingControllerParameters.getSteppingParameters().getInPlaceWidth();
      double min = 0.20;
      double max = 0.50;
      continuousStepGenerator.setStepWidths(inPlaceStepWidth, min, max);
      continuousStepGenerator.setMaxStepLength(0.50);
      continuousStepGenerator.setSupportFootBasedFootstepAdjustment(false);
      continuousStepGenerator.setFootstepMessenger(this::prepareFootsteps);
      continuousStepGenerator.setFootPoseProvider(robotSide -> new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide)));
      packetCommunicator.attachListener(FootstepStatusMessage.class, continuousStepGenerator::consumeFootstepStatus);

      swingHeight = messager.createInput(WalkingSwingHeight, 0.05);
      swingDuration = messager.createInput(WalkingSwingDuration, walkingControllerParameters.getDefaultSwingTime());
      transferDuration = messager.createInput(WalkingTransferDuration, walkingControllerParameters.getDefaultTransferTime());
      trajectoryDuration = messager.createInput(WalkingTrajectoryDuration, 1.0);

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

      messager.registerTopicListener(ButtonXState, state -> {
         if (state == ButtonState.PRESSED)
         {
            if (isInDoubleSupport.getValue())
               flamingoHomeStance(RobotSide.LEFT);
            else if (!isLeftFootInSupport.get())
               putFootDown(RobotSide.LEFT);
         }
      });
      messager.registerTopicListener(ButtonBState, state -> {
         if (state == ButtonState.PRESSED)
         {
            if (isInDoubleSupport.getValue())
               flamingoHomeStance(RobotSide.RIGHT);
            else if (!isRightFootInSupport.get())
               putFootDown(RobotSide.RIGHT);
         }
      });

      messager.registerTopicListener(ButtonYState, state -> {
         if (isInDoubleSupport.getValue())
            return;

         RobotSide kickSide = isRightFootInSupport.get() ? RobotSide.LEFT : RobotSide.RIGHT;

         if (state == ButtonState.PRESSED)
            kick(kickSide);
         else if (state == ButtonState.RELEASED)
            flamingoHomeStance(kickSide);
      });

      messager.registerTopicListener(ButtonLeftBumperState, state -> {
         if (state == ButtonState.PRESSED)
            sendArmHomeConfiguration(RobotSide.values);
      });
      messager.registerTopicListener(ButtonRightBumperState, state -> startWalking(state == ButtonState.PRESSED));
      messager.registerTopicListener(ButtonRightBumperState, state -> stopWalking(state == ButtonState.RELEASED));
      messager.registerJavaFXSyncedTopicListener(LeftStickYAxis, this::updateForwardVelocity);
      messager.registerJavaFXSyncedTopicListener(LeftStickXAxis, this::updateLateralVelocity);
      messager.registerJavaFXSyncedTopicListener(RightStickXAxis, this::updateTurningVelocity);

      packetCommunicator.attachListener(WalkingControllerFailureStatusMessage.class, packet -> stopWalking(true));
      messager.registerTopicListener(ButtonSelectState, state -> sendLowLevelControlModeRequest(AtlasLowLevelControlMode.FREEZE));
      messager.registerTopicListener(ButtonStartState, state -> sendLowLevelControlModeRequest(AtlasLowLevelControlMode.STAND_PREP));
      packetCommunicator.attachListener(CapturabilityBasedStatus.class, status -> {
         isLeftFootInSupport.set(!status.getLeftFootSupportPolygon2d().isEmpty());
         isRightFootInSupport.set(!status.getRightFootSupportPolygon2d().isEmpty());
      });
   }

   private void updateForwardVelocity(double alpha)
   {
      double minMaxVelocity = 0.30;
      forwardVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void updateLateralVelocity(double alpha)
   {
      double minMaxVelocity = 0.30;
      lateralVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void updateTurningVelocity(double alpha)
   {
      double minMaxVelocity = Math.PI / 4.0;
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
      packetCommunicator.send(pauseWalkingMessage);
   }

   public void sendLowLevelControlModeRequest(AtlasLowLevelControlMode mode)
   {
      stopWalking(true);
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
      //      footstepDataListMessage.setAreFootstepsAdjustable(true);
      footstepsToSendReference.set(new FootstepDataListMessage(footstepDataListMessage));
   }

   private void sendFootsteps()
   {
      FootstepDataListMessage footstepsToSend = footstepsToSendReference.getAndSet(null);
      if (footstepsToSend != null && isWalking.get())
      {
         packetCommunicator.send(footstepsToSend);
      }
      if (!isWalking.get())
         sendPauseMessage();
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

   private void processPunch(RobotSide robotSide, ButtonState state)
   {
      if (state == ButtonState.PRESSED)
         sendArmStraightConfiguration(robotSide);
      else
         sendArmHomeConfiguration(robotSide);
   }

   private void sendArmHomeConfiguration(RobotSide... robotSides)
   {
      for (RobotSide robotSide : robotSides)
      {
         double[] jointAngles = new double[7];
         int index = 0;
         jointAngles[index++] = robotSide.negateIfRightSide(0.785398); // ArmJointName.SHOULDER_YAW        
         jointAngles[index++] = robotSide.negateIfRightSide(-0.52379); // ArmJointName.SHOULDER_ROLL       
         jointAngles[index++] = 2.33708; // ArmJointName.ELBOW_PITCH         
         jointAngles[index++] = robotSide.negateIfRightSide(2.35619); // ArmJointName.ELBOW_ROLL          
         jointAngles[index++] = -0.337807; // ArmJointName.FIRST_WRIST_PITCH   
         jointAngles[index++] = robotSide.negateIfRightSide(0.207730); // ArmJointName.WRIST_ROLL          
         jointAngles[index++] = -0.026599; // ArmJointName.SECOND_WRIST_PITCH
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, 3.0, jointAngles);
         packetCommunicator.send(message);
      }
   }

   private void sendArmStraightConfiguration(RobotSide robotSide)
   {
      double[] jointAngles0 = new double[7];
      int index = 0;
      jointAngles0[index++] = robotSide.negateIfRightSide(-0.2); // ArmJointName.SHOULDER_YAW        
      jointAngles0[index++] = robotSide.negateIfRightSide(-0.17); // ArmJointName.SHOULDER_ROLL       
      jointAngles0[index++] = 1.4; // ArmJointName.ELBOW_PITCH         
      jointAngles0[index++] = robotSide.negateIfRightSide(1.8); // ArmJointName.ELBOW_ROLL          
      jointAngles0[index++] = -0.337807; // ArmJointName.FIRST_WRIST_PITCH   
      jointAngles0[index++] = robotSide.negateIfRightSide(0.207730); // ArmJointName.WRIST_ROLL          
      jointAngles0[index++] = -0.026599; // ArmJointName.SECOND_WRIST_PITCH

      double[] jointAngles1 = new double[7];
      index = 0;
      jointAngles1[index++] = robotSide.negateIfRightSide(-1.2); // ArmJointName.SHOULDER_YAW        
      jointAngles1[index++] = robotSide.negateIfRightSide(-0.0); // ArmJointName.SHOULDER_ROLL       
      jointAngles1[index++] = 1.8; // ArmJointName.ELBOW_PITCH         
      jointAngles1[index++] = robotSide.negateIfRightSide(0.6); // ArmJointName.ELBOW_ROLL          
      jointAngles1[index++] = -0.337807; // ArmJointName.FIRST_WRIST_PITCH   
      jointAngles1[index++] = robotSide.negateIfRightSide(0.207730); // ArmJointName.WRIST_ROLL          
      jointAngles1[index++] = -0.026599; // ArmJointName.SECOND_WRIST_PITCH
      ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryDuration.get() / 2.0, jointAngles0);
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = message.getJointspaceTrajectory().getJointTrajectoryMessages();
      for (int i = 0; i < jointTrajectoryMessages.size(); i++)
      {
         TrajectoryPoint1DMessage trajectoryPoint1DMessage = createTrajectoryPoint1DMessage(trajectoryDuration.get(), jointAngles1[i], 0.0);
         jointTrajectoryMessages.get(i).getTrajectoryPoints().add().set(trajectoryPoint1DMessage);
      }
      packetCommunicator.send(message);
   }

   private void flamingoHomeStance(RobotSide robotSide)
   {
      FramePose3D footPose = new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.0, robotSide.negateIfRightSide(inPlaceStepWidth), 0.15);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryDuration.get(), footPose);
      packetCommunicator.send(message);
   }

   private void putFootDown(RobotSide robotSide)
   {
      if (isFootInSupport.get(robotSide).get())
         return;

      FramePose3D footPose = new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.0, robotSide.negateIfRightSide(inPlaceStepWidth), 0.0);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryDuration.get(), footPose);
      packetCommunicator.send(message);

      packetCommunicator.send(HumanoidMessageTools.createFootLoadBearingMessage(robotSide, LoadBearingRequest.LOAD));
   }

   private void kick(RobotSide robotSide)
   {
      if (isFootInSupport.get(robotSide).get())
         return;

      FramePose3D footPose = new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.60, robotSide.negateIfRightSide(inPlaceStepWidth), 0.35);
      footPose.appendPitchRotation(-0.8);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, 0.33 * trajectoryDuration.get(), footPose);
      packetCommunicator.send(message);
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
