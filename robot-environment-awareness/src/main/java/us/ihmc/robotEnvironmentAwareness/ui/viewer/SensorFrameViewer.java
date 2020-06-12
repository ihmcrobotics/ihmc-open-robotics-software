package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class SensorFrameViewer<T extends Packet<T>> extends AnimationTimer
{
   private static final int TRAJECTORY_RADIAL_RESOLUTION = 16;
   private static final double TRAJECTORY_MESH_RADIUS = 0.01;

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

   public SensorFrameViewer(REAUIMessager uiMessager, Topic<T> messageState, Topic<Integer> numberOfFramesTopic, Function<T, SensorFrame> function,
                            Topic<Boolean> clearTopic)
   {
      this.function = function;
      if (numberOfFramesTopic == null)
         numberOfFramesToShow = new AtomicReference<Integer>(DEFAULT_NUMBER_OF_FRAMES);
      else
         numberOfFramesToShow = uiMessager.createInput(numberOfFramesTopic, 10);
      uiMessager.registerTopicListener(clearTopic, (c) -> clear());

      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      sensorCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      sensorCoordinateSystem.getTransforms().add(sensorPose);
      affineRoot.getChildren().add(sensorCoordinateSystem);

      root.getChildren().add(affineRoot);
      root.getChildren().add(historyRoot);
      root.setMouseTransparent(true);

      latestMessage = uiMessager.createInput(messageState);
      uiMessager.registerModuleMessagerStateListener(isMessagerOpen ->
      {
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

      int numberOfSensorFrames = sensorOriginHistory.size();
      meshBuilder.clear();
      Point3D32 point = new Point3D32();
      Point3D[] sensorPoseTrajectoryPoints = new Point3D[numberOfSensorFrames];
      for (int i = 0; i < numberOfSensorFrames; i++)
      {
         sensorOriginHistory.get(i).getOrigin(point);
         double confidence = sensorOriginHistory.get(i).confidence;
         if (confidence < 0)
            confidence = 0.0;
         int redScaler = (int) (0xFF * (1 - confidence));
         int greenScaler = (int) (0xFF * confidence);
         Color confidenceColor = Color.rgb(redScaler, greenScaler, 0);
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(ORIGIN_POINT_SIZE), point, confidenceColor);

         sensorPoseTrajectoryPoints[i] = sensorOriginHistory.get(i).getPointCopy();
      }
      if (numberOfSensorFrames > 1)
      {
         SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(numberOfSensorFrames,
                                                                                                              TRAJECTORY_RADIAL_RESOLUTION,
                                                                                                              TRAJECTORY_MESH_RADIUS);
         segmentedLine3DMeshGenerator.compute(sensorPoseTrajectoryPoints);
         for (MeshDataHolder mesh : segmentedLine3DMeshGenerator.getMeshDataHolders())
            meshBuilder.addMesh(mesh, Color.ALICEBLUE);
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

   public static Function<StampedPosePacket, SensorFrame> createStampedPosePacketSensorFrameExtractor()
   {
      return message ->
      {
         Pose3D pose = message.getPose();
         return new SensorFrame(pose.getPosition(), pose.getOrientation(), message.getConfidenceFactor());
      };
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

      public Point3D getPointCopy()
      {
         return new Point3D(affine.getTx(), affine.getTy(), affine.getTz());
      }
   }
}
