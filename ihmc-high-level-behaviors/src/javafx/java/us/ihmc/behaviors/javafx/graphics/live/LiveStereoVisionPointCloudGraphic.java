package us.ihmc.behaviors.javafx.graphics.live;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javafx.PrivateAnimationTimer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.concurrent.atomic.AtomicReference;

public class LiveStereoVisionPointCloudGraphic extends Group
{
   private static final float SCAN_POINT_SIZE = 0.0075f;
   private static final int PALLETE_SIZE_FOR_MESH_BUILDER = 2048;

   private final JavaFXMultiColorMeshBuilder meshBuilder;
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::updateScene);

   protected final AtomicReference<MeshView> scanMeshToRender = new AtomicReference<>(null);
   private final ResettableExceptionHandlingExecutorService threadQueue;

   public LiveStereoVisionPointCloudGraphic(ROS2Node ros2Node, ROS2Topic<?> topic)
   {
      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(PALLETE_SIZE_FOR_MESH_BUILDER));
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      if (topic.getType().equals(LidarScanMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
      }
      animationTimer.start();
   }

   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
   {
      threadQueue.clearQueueAndExecute(() ->
      {
         Point3D32[] points = StereoPointCloudCompression.decompressPointCloudToArray32(message);
         int[] colors = StereoPointCloudCompression.decompressColorsToIntArray(message);
         buildMesh(points, colors);
      });
   }

   private void queueRenderLidarScan(LidarScanMessage message)
   {
      threadQueue.clearQueueAndExecute(() ->
      {
         int numberOfPoints = message.getNumberOfPoints();
         Point3D32[] points = new Point3D32[numberOfPoints];
         int[] colors = new int[numberOfPoints];
         LidarPointCloudCompression.decompressPointCloud(message.getScan(), numberOfPoints, (i, x, y, z) ->
         {
            points[i] = new Point3D32();
            points[i].setX(x);
            points[i].setY(y);
            points[i].setZ(z);
         });

         buildMesh(points, colors);
      });
   }

   public void updateScene(long now)
   {
      MeshView newScanMeshView = scanMeshToRender.getAndSet(null);

      if (newScanMeshView != null)
      {
         getChildren().clear();
         getChildren().add(newScanMeshView);
      }
   }

   public void buildMesh(Point3D32[] points, int[] colors)
   {
      meshBuilder.clear();
      int numberOfScanPoints = points.length;

      for (int i = 0; i < numberOfScanPoints; i++)
      {
         Color color = intToColor(colors[i]);
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), points[i], color);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      scanMeshToRender.set(scanMeshView);
      meshBuilder.clear();
   }

   public void clear()
   {
      getChildren().clear();
   }

   public static Color intToColor(int value)
   {
      int r = value >> 16 & 0xFF;
      int g = value >> 8 & 0xFF;
      int b = value >> 0 & 0xFF;
      return Color.rgb(r, g, b);
   }
}
