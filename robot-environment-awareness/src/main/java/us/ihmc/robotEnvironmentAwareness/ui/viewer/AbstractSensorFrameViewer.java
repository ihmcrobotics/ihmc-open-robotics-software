package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.communication.packets.Packet;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public abstract class AbstractSensorFrameViewer<T extends Packet<T>> extends AnimationTimer
{
   protected final JavaFXCoordinateSystem sensorCoordinateSystem;
   protected final Affine sensorPose = new Affine();

   protected final AtomicReference<Affine> lastAffine = new AtomicReference<>();

   private final Group root = new Group();

   public AbstractSensorFrameViewer(REAUIMessager uiMessager, Topic<T> messageState)
   {
      sensorCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      sensorCoordinateSystem.getTransforms().add(sensorPose);
      root.getChildren().add(sensorCoordinateSystem);
      root.setMouseTransparent(true);

      uiMessager.registerTopicListener(messageState, this::handleMessage);
      uiMessager.registerModuleMessagerStateListener(isMessagerOpen -> {
         if (isMessagerOpen)
            start();
         else
            stop();
      });
   }

   @Override
   public void handle(long now)
   {
      Affine affine = lastAffine.getAndSet(null);
      if (affine != null)
         sensorPose.setToTransform(affine);
   }

   public Node getRoot()
   {
      return root;
   }

   public abstract void handleMessage(T message);
}
