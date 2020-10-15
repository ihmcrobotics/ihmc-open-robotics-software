package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PointCloudAnchorPaneController;

public class StereoVisionPointCloudViewer extends AbstractSourceViewer<StereoVisionPointCloudMessage>
{
   private final AtomicReference<Integer> sizeOfPointCloud;

   public StereoVisionPointCloudViewer(Topic<StereoVisionPointCloudMessage> messageState, REAUIMessager uiMessager, Topic<Boolean> enableTopic, Topic<Boolean> clearTopic,
                                       Topic<Integer> sizeTopic)
   {
      super(messageState, uiMessager, enableTopic, clearTopic);
      sizeOfPointCloud = uiMessager.createInput(sizeTopic, PointCloudAnchorPaneController.initialSizeOfPointCloud);
   }

   public void render()
   {
      MeshView newScanMeshView = scanMeshToRender.getAndSet(null);

      if (clear.getAndSet(false))
         children.clear();

      if (!enable.get())
         return;

      if (newScanMeshView != null)
      {
         children.clear();
         children.add(newScanMeshView);
      }
   }

   @Override
   public void unpackPointCloud(StereoVisionPointCloudMessage message)
   {
      Point3D32[] pointcloud = PointCloudCompression.decompressPointCloudToArray32(message);
      int[] colors = PointCloudCompression.decompressColorsToIntArray(message);

      meshBuilder.clear();
      int numberOfScanPoints = pointcloud.length;
      int sizeOfPointCloudToVisualize = Math.min(numberOfScanPoints, sizeOfPointCloud.get());

      Random random = new Random();
      for (int i = 0; i < sizeOfPointCloudToVisualize; i++)
      {
         int indexToVisualize;
         if (numberOfScanPoints < sizeOfPointCloud.get())
            indexToVisualize = i;
         else
            indexToVisualize = random.nextInt(numberOfScanPoints);

         Color color = intToColor(colors[indexToVisualize]);

         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), pointcloud[indexToVisualize], color);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      scanMeshToRender.set(scanMeshView);
      meshBuilder.clear();
   }

   public static javafx.scene.paint.Color intToColor(int value)
   {
      int r = value >> 16 & 0xFF;
      int g = value >> 8 & 0xFF;
      int b = value >> 0 & 0xFF;
      return Color.rgb(r, g, b);
   }
   
   public static int colorToInt(javafx.scene.paint.Color color)
   {
      int rgb = (int) (color.getRed() * 0xFF) << 16 | (int) (color.getGreen() * 0xFF) << 8 | (int) (color.getBlue() * 0xFF) << 0;
      return rgb;
   }
}
