package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class StereoVisionPointCloudViewer extends AbstractSourceViewer<StereoVisionPointCloudMessage>
{
   public StereoVisionPointCloudViewer(Topic<StereoVisionPointCloudMessage> messageState, REAUIMessager uiMessager, int palleteSizeForMeshBuilder)
   {
      super(messageState, uiMessager, palleteSizeForMeshBuilder);
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
      Point3D32 scanPoint = new Point3D32();
      meshBuilder.clear();
      int numberOfScanPoints = message.getPointCloud().size() / 3;
      for (int i = 0; i < numberOfScanPoints; i++)
      {
         int colorValue = message.getColors().get(i);
         int r = colorValue >> 16 & 0xFF;
         int g = colorValue >> 8 & 0xFF;
         int b = colorValue >> 0 & 0xFF;
         Color color = Color.rgb(r, g, b);

         MessageTools.unpackScanPoint(message, i, scanPoint);

         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, color);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      scanMeshToRender.set(scanMeshView);
      meshBuilder.clear();
   }
}
