package us.ihmc.humanoidBehaviors.ui.graphics.live;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.atomic.AtomicReference;

public class LiveStereoVisionPointCloudGraphic extends Group
{
   private static final float SCAN_POINT_SIZE = 0.0075f;
   private static final int PALLETE_SIZE_FOR_MESH_BUILDER = 2048;

   private final JavaFXMultiColorMeshBuilder meshBuilder;
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::updateScene);

   protected final AtomicReference<MeshView> scanMeshToRender = new AtomicReference<>(null);
   private final SingleThreadSizeOneQueueExecutor threadQueue;

   public LiveStereoVisionPointCloudGraphic(ROS2Node ros2Node, ROS2Topic<StereoVisionPointCloudMessage> topic)
   {
      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(PALLETE_SIZE_FOR_MESH_BUILDER));

      threadQueue = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      new IHMCROS2Callback<>(ros2Node, topic, this::queueRender);

      animationTimer.start();
   }

   private void queueRender(StereoVisionPointCloudMessage message)
   {
      threadQueue.queueExecution(() -> buildMesh(message));
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

   public void buildMesh(StereoVisionPointCloudMessage message)
   {
      Point3D32[] points = PointCloudCompression.decompressPointCloudToArray32(message);
      int[] colors = PointCloudCompression.decompressColorsToIntArray(message);

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
