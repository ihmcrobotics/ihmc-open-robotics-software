package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.LidarScanMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class LidarFrameViewer extends AnimationTimer
{
   private final JavaFXCoordinateSystem lidarCoordinateSystem;
   private final Affine lidarPose = new Affine();

   private final AtomicReference<Affine> lastAffine = new AtomicReference<>();

   private final Group root = new Group();
   public LidarFrameViewer(REAUIMessager uiMessager)
   {
      lidarCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      lidarCoordinateSystem.getTransforms().add(lidarPose);
      root.getChildren().add(lidarCoordinateSystem);
      root.setMouseTransparent(true);

      uiMessager.registerTopicListener(REAModuleAPI.LidarScanState, this::handleMessage);
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
         lidarPose.setToTransform(affine);
   }

   private void handleMessage(LidarScanMessage lidarScanMessage)
   {
      if (lidarScanMessage == null)
         return;
      Quaternion orientation = lidarScanMessage.getLidarOrientation();
      Point3D position = lidarScanMessage.getLidarPosition();
      lastAffine.set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));
   }

   public Node getRoot()
   {
      return root;
   }
}
