package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class SensorFrameViewer<T extends Packet<T>> extends AnimationTimer
{
   private final AtomicReference<T> latestMessage;

   protected final JavaFXCoordinateSystem sensorCoordinateSystem;
   protected final Affine sensorPose = new Affine();

   private static final float ORIGIN_POINT_SIZE = 0.05f;
   private static final int DEFAULT_NUMBER_OF_FRAMES = 1;
   private final LinkedList<SensorFrame> sensorOriginHistory = new LinkedList<SensorFrame>();
   private final AtomicReference<Integer> numberOfFramesToShow;

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final Group root = new Group();
   private final Group affineRoot = new Group();
   private final Group historyRoot = new Group();

   private Function<T, SensorFrame> function;

   public SensorFrameViewer(REAUIMessager uiMessager, Topic<T> messageState, Topic<Integer> numberOfFramesTopic, Function<T, SensorFrame> function)
   {
      this.function = function;
      if (numberOfFramesTopic == null)
         numberOfFramesToShow = new AtomicReference<Integer>(DEFAULT_NUMBER_OF_FRAMES);
      else
         numberOfFramesToShow = uiMessager.createInput(numberOfFramesTopic, 10); //REAModuleAPI.UINavigationFrames
      uiMessager.registerTopicListener(REAModuleAPI.UISensorPoseHistoryClear, (c) -> clear());

      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      sensorCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      sensorCoordinateSystem.getTransforms().add(sensorPose);
      affineRoot.getChildren().add(sensorCoordinateSystem);

      root.getChildren().add(affineRoot);
      root.getChildren().add(historyRoot);
      root.setMouseTransparent(true);

      latestMessage = uiMessager.createInput(messageState);
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
      if (latestMessage.get() == null)
         return;

      SensorFrame latestSensorFrame = function.apply(latestMessage.getAndSet(null));

      Affine affine = latestSensorFrame.getAffine();
      if (affine != null)
         sensorPose.setToTransform(affine);

      sensorOriginHistory.add(latestSensorFrame);
      if (sensorOriginHistory.size() == numberOfFramesToShow.get() + 1)
         sensorOriginHistory.removeFirst();

      if (sensorOriginHistory.size() == 0)
         return;

      meshBuilder.clear();
      Point3D32 point = new Point3D32();
      for (int i = 0; i < sensorOriginHistory.size(); i++)
      {
         sensorOriginHistory.get(i).getOrigin(point);
         int redScaler = (int) (0xFF * (1 - (sensorOriginHistory.get(i).confidence)));
         int greenScaler = (int) (0xFF * (sensorOriginHistory.get(i).confidence));
         Color confidenceColor = Color.rgb(redScaler, greenScaler, 0);
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(ORIGIN_POINT_SIZE), point, confidenceColor);
      }
      MeshView historyMeshView = new MeshView(meshBuilder.generateMesh());
      historyMeshView.setMaterial(meshBuilder.generateMaterial());
      meshBuilder.clear();

      if (historyMeshView != null)
      {
         historyRoot.getChildren().clear();
         historyRoot.getChildren().add(historyMeshView);
      }
   }

   private void clear()
   {
      sensorOriginHistory.clear();
      historyRoot.getChildren().clear();
   }

   public Node getRoot()
   {
      return root;
   }

   public static Function<LidarScanMessage, SensorFrame> createLidarScanSensorFrameExtractor()
   {
      return message -> new SensorFrame(message.getLidarPosition(), message.getLidarOrientation(), message.getSensorPoseConfidence());
   }

   public static Function<StereoVisionPointCloudMessage, SensorFrame> createStereoVisionSensorFrameExtractor()
   {
      return message -> new SensorFrame(message.getSensorPosition(), message.getSensorOrientation(), message.getSensorPoseConfidence());
   }

   public static class SensorFrame
   {
      private final Affine affine;
      private final double confidence;

      public SensorFrame(Point3DBasics position, QuaternionBasics orientation, double confidence)
      {
         this.affine = JavaFXTools.createAffineFromOrientation3DAndTuple(orientation, position);
         this.confidence = confidence;
      }

      public Affine getAffine()
      {
         return affine;
      }

      void getOrigin(Point3D32 pointToPack)
      {
         pointToPack.set(affine.getTx(), affine.getTy(), affine.getTz());
      }
   }

}
