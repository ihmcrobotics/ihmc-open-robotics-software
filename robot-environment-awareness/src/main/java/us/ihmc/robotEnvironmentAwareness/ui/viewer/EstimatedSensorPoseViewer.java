package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.EnumMap;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeBuffer.SensorPoseSourceType;

public class EstimatedSensorPoseViewer extends AnimationTimer
{
   protected static final float ORIGIN_POINT_SIZE = 0.05f;

   private final JavaFXCoordinateSystem sensorCoordinateSystem;
   private final AtomicReference<SensorPoseSourceType> sensorPoseSourceType;
   private final EnumMap<SensorPoseSourceType, AtomicReference<Affine>> latestSensorPoseMapToMessages = new EnumMap<>(SensorPoseSourceType.class);
   private final Affine sensorPose = new Affine();

   private final AtomicReference<Integer> numberOfFramesToShow;

   private final Group root = new Group();
   private final Group affineRoot = new Group();
   private final Group historyRoot = new Group();

   private final AtomicReference<MeshView> historyMeshToRender = new AtomicReference<>(null);

   private final LinkedList<SensorOrigin> sensorOriginHistory = new LinkedList<SensorOrigin>();

   private final AtomicReference<Boolean> enable;
   private final JavaFXMultiColorMeshBuilder meshBuilder;

   public EstimatedSensorPoseViewer(REAUIMessager uiMessager)
   {
      sensorPoseSourceType = uiMessager.createInput(REAModuleAPI.SensorPoseSourceType, SensorPoseSourceType.STEREO);
      numberOfFramesToShow = uiMessager.createInput(REAModuleAPI.UINavigationFrames, 10);

      for (SensorPoseSourceType sourceType : SensorPoseSourceType.values())
         latestSensorPoseMapToMessages.put(sourceType, new AtomicReference<Affine>());

      uiMessager.registerTopicListener(REAModuleAPI.UINavigationClear, (c) -> clear());
      enable = uiMessager.createInput(REAModuleAPI.UINavigationShow, false);

      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      sensorCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      sensorCoordinateSystem.getTransforms().add(sensorPose);
      affineRoot.getChildren().add(sensorCoordinateSystem);

      root.getChildren().add(affineRoot);
      root.getChildren().add(historyRoot);
      root.setMouseTransparent(true);

      uiMessager.registerTopicListener(REAModuleAPI.LidarScanState, this::handleMessage);
      uiMessager.registerTopicListener(REAModuleAPI.StereoVisionPointCloudState, this::handleMessage);
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
      if (!enable.get())
         return;

      SensorPoseSourceType selectedSourceType = sensorPoseSourceType.get();
      Affine affine = latestSensorPoseMapToMessages.get(selectedSourceType).getAndSet(null);

      if (affine != null)
         sensorPose.setToTransform(affine);

      MeshView newHistoryMeshToRender = historyMeshToRender.getAndSet(null);

      if (newHistoryMeshToRender != null)
      {
         historyRoot.getChildren().clear();
         historyRoot.getChildren().add(newHistoryMeshToRender);
      }
   }

   private void handleMessage(LidarScanMessage message)
   {
      if (message == null)
         return;
      Quaternion orientation = message.getLidarOrientation();
      Point3D position = message.getLidarPosition();

      latestSensorPoseMapToMessages.get(SensorPoseSourceType.LIDAR_SCAN).set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));
      addHistory(1.0, position);
   }

   private void handleMessage(StereoVisionPointCloudMessage message)
   {
      if (message == null)
         return;
      Quaternion orientation = message.getSensorOrientation();
      Point3D position = message.getSensorPosition();
      latestSensorPoseMapToMessages.get(SensorPoseSourceType.STEREO).set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));
      addHistory(0.0, position);
   }

   private void addHistory(double confidence, Point3D newHistory)
   {
      sensorOriginHistory.add(new SensorOrigin(newHistory, confidence));
      if (sensorOriginHistory.size() == numberOfFramesToShow.get())
         sensorOriginHistory.removeFirst();

      meshBuilder.clear();
      Point3D32 point = new Point3D32();
      for (int i = 0; i < sensorOriginHistory.size(); i++)
      {
         point.set(sensorOriginHistory.get(i).origin);
         int redScaler = (int) (0xFF * (1 - (sensorOriginHistory.get(i).confidence)));
         int greenScaler = (int) (0xFF * (sensorOriginHistory.get(i).confidence));
         Color confidenceColor = Color.rgb(redScaler, greenScaler, 0);
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(ORIGIN_POINT_SIZE), point, confidenceColor);
      }
      MeshView historyMeshView = new MeshView(meshBuilder.generateMesh());
      historyMeshView.setMaterial(meshBuilder.generateMaterial());
      historyMeshToRender.set(historyMeshView);
      meshBuilder.clear();
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

   class SensorOrigin
   {
      private final Point3DBasics origin;
      private final double confidence;

      SensorOrigin(Point3D origin, double confidence)
      {
         this.origin = origin;
         this.confidence = confidence;
      }
   }
}
