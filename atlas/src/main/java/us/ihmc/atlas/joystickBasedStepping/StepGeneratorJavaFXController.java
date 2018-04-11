package us.ihmc.atlas.joystickBasedStepping;

import java.io.IOException;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public class StepGeneratorJavaFXController
{
   private final SideDependentList<Color> footColors = new SideDependentList<Color>(Color.CRIMSON, Color.YELLOWGREEN);

   private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
   private final DoubleProperty headingVelocity = new SimpleDoubleProperty(this, "headingVelocityProperty", 0.0);
   private final Vector2DProperty desiredVelocityProperty = new Vector2DProperty(this, "desiredVelocity", new Vector2D());
   private final AnimationTimer animationTimer;
   private final PacketCommunicator packetCommunicator;
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final AtomicReference<List<Node>> footstepsToVisualizeReference = new AtomicReference<>(null);

   private final Group rootNode = new Group();

   private final JoystickPlanarVelocityProvider joystickPlanarVelocityProvider;

   private final Joystick joystick;

   public StepGeneratorJavaFXController(WalkingControllerParameters walkingControllerParameters, PacketCommunicator packetCommunicator,
                                            JavaFXRobotVisualizer javaFXRobotVisualizer)
   {
      this.packetCommunicator = packetCommunicator;
      continuousStepGenerator.setDesiredHeadingProvider(() -> headingVelocity.get());
      continuousStepGenerator.setDesiredVelocityProvider(() -> desiredVelocityProperty.get());
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
            headingVelocity.set(joystickPlanarVelocityProvider.getTurningVelocity());
            desiredVelocityProperty.set(joystickPlanarVelocityProvider.getVelocity());

            update();

            List<Node> footstepsToVisualize = footstepsToVisualizeReference.getAndSet(null);
            ObservableList<Node> children = rootNode.getChildren();
//
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

      try
      {
         joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
         joystick.setStandalone();
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1));
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1));
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1));
      }
      catch (IOException e)
      {
         throw new RuntimeException("error opening joystick: " + e);
      }

      joystickPlanarVelocityProvider = new JoystickPlanarVelocityProvider(joystick, 0.3, 0.3, Math.PI / 3.0);

      joystick.addJoystickEventListener(event -> {
         if (event.getComponent().getIdentifier() == XBoxOneMapping.A.getIdentifier())
            continuousStepGenerator.startWalking();
         else if (event.getComponent().getIdentifier() == XBoxOneMapping.B.getIdentifier())
         {
            continuousStepGenerator.stopWalking();
            PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
            pauseWalkingMessage.setPause(true);
            packetCommunicator.send(pauseWalkingMessage);
         }
      });
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
      MeshDataHolder polygon = MeshDataGenerator.ExtrudedPolygon(footPolygon, 0.05);
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
      joystick.shutdown();
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   public class JoystickPlanarVelocityProvider
   {
      private double xdot;
      private double ydot;
      private double adot;

      public JoystickPlanarVelocityProvider(Joystick joystick, double xdotMax, double ydotMax, double adotMax)
      {
         joystick.addJoystickEventListener(event -> {
            if (event.getComponent().getIdentifier() == XBoxOneMapping.LEFT_STICK_Y.getIdentifier())
               xdot = xdotMax * event.getValue();
            else if (event.getComponent().getIdentifier() == XBoxOneMapping.LEFT_STICK_X.getIdentifier())
               ydot = ydotMax * event.getValue();
            else if (event.getComponent().getIdentifier() == XBoxOneMapping.RIGHT_STICK_X.getIdentifier())
               adot = adotMax * event.getValue();
         });
      }

      public Vector2D getVelocity()
      {
         return new Vector2D(xdot, ydot);
      }

      public double getForwardVelocity()
      {
         return xdot;
      }

      public double getLateralVelocity()
      {
         return ydot;
      }

      public double getTurningVelocity()
      {
         return adot;
      }
   }
}
