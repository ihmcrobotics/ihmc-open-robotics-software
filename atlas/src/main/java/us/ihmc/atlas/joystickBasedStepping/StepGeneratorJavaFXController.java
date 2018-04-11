package us.ihmc.atlas.joystickBasedStepping;

import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonAState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonBState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.LeftStickXAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.LeftStickYAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.LeftTriggerAxis;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
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
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class StepGeneratorJavaFXController
{
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

   public StepGeneratorJavaFXController(JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters,
                                        PacketCommunicator packetCommunicator, JavaFXRobotVisualizer javaFXRobotVisualizer)
   {
      this.packetCommunicator = packetCommunicator;
      continuousStepGenerator.setNumberOfFootstepsToPlan(10);
      continuousStepGenerator.setDesiredHeadingProvider(() -> headingVelocityProperty.get());
      continuousStepGenerator.setDesiredVelocityProvider(() -> new Vector2D(forwardVelocityProperty.get(), lateralVelocityProperty.get()));
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.setSupportFootBasedFootstepAdjustment();
      continuousStepGenerator.setFootstepMessenger(this::visualizeFootstepsAndSend);
      continuousStepGenerator.setFootPoseProvider(robotSide -> new FramePose3D(javaFXRobotVisualizer.getFullRobotModel().getSoleFrame(robotSide)));
      packetCommunicator.attachListener(FootstepStatusMessage.class, continuousStepGenerator::consumeFootstepStatus);

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            update();

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

      messager.registerTopicListener(ButtonAState, state -> startWalking(true));
      messager.registerTopicListener(ButtonBState, state -> stopWalking(true));
      messager.registerJavaFXSyncedTopicListener(LeftStickYAxis, this::updateForwardVelocity);
      messager.registerJavaFXSyncedTopicListener(LeftStickXAxis, this::updateLateralVelocity);
      messager.registerJavaFXSyncedTopicListener(LeftTriggerAxis, this::updateHeadingVelocity);
   }

   private void updateForwardVelocity(double alpha)
   {
      double minMaxVelocity = 0.3;
      forwardVelocityProperty.set(minMaxVelocity * alpha);
   }

   private void updateLateralVelocity(double alpha)
   {
      double minMaxVelocity = 0.3;
      lateralVelocityProperty.set(minMaxVelocity * alpha);
   }

   private void updateHeadingVelocity(double alpha)
   {
      double minMaxVelocity = Math.PI / 3.0;
      headingVelocityProperty.set(-minMaxVelocity * alpha);
   }

   private void startWalking(boolean confirm)
   {
      if (confirm)
      {
         continuousStepGenerator.startWalking();
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

   private void visualizeFootstepsAndSend(FootstepDataListMessage footstepDataListMessage)
   {
      List<Node> footstepNode = new ArrayList<>();
      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
         footstepNode.add(createFootstep(footstepDataListMessage.getFootstepDataList().get(i)));
      footstepsToVisualizeReference.set(footstepNode);

      packetCommunicator.send(footstepDataListMessage);
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

   private void update()
   {
      continuousStepGenerator.update(Double.NaN);
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
