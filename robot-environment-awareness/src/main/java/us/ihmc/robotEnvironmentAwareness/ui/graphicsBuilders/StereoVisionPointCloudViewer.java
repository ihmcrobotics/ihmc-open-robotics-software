package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class StereoVisionPointCloudViewer extends AbstractSourceViewer<StereoVisionPointCloudMessage>
{
   public StereoVisionPointCloudViewer(Topic<StereoVisionPointCloudMessage> messageState, REAUIMessager uiMessager)
   {
      super(messageState, uiMessager);
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
         Color color = intToColor(colorValue);

         MessageTools.unpackScanPoint(message, i, scanPoint);

         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, color);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      scanMeshToRender.set(scanMeshView);
      meshBuilder.clear();
   }

   @Override
   protected Topic<Boolean> createEnableInput()
   {
      return REAModuleAPI.UIStereoVisionPointCloudShow;
   }

   @Override
   protected Topic<Boolean> createClearInput()
   {
      return REAModuleAPI.UIStereoVisionPointCloudClear;
   }

   public static javafx.scene.paint.Color intToColor(int value)
   {
      int r = value >> 16 & 0xFF;
      int g = value >> 8 & 0xFF;
      int b = value >> 0 & 0xFF;
      return Color.rgb(r, g, b);
   }
}
