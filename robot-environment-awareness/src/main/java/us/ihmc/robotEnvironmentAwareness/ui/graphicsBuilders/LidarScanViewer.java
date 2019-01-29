package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.LidarScanMessage;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

/**
 * Created by adrien on 11/20/16.
 */
public class LidarScanViewer extends AbstractSourceViewer<LidarScanMessage>
{
   private final AtomicReference<Integer> numberOfScans;
   private final AtomicInteger currentScanIndex = new AtomicInteger(0);

   private static final Material defaultMaterial = new PhongMaterial(Color.DARKRED);

   public LidarScanViewer(Topic<LidarScanMessage> messageState, REAUIMessager uiMessager)
   {
      super(messageState, uiMessager);
      numberOfScans = uiMessager.createInput(REAModuleAPI.UILidarScanSize, 50);
   }

   @Override
   public void render()
   {
      MeshView newScanMeshView = scanMeshToRender.getAndSet(null);

      if (clear.getAndSet(false))
      {
         children.clear();
         currentScanIndex.set(0);
      }

      while (children.size() > numberOfScans.get())
         children.remove(children.size() - 1);

      if (!enable.get())
         return;

      if (newScanMeshView != null)
      {
         if (children.size() <= currentScanIndex.get())
            children.add(newScanMeshView);
         else
            children.set(currentScanIndex.get(), newScanMeshView);

         for (int i = currentScanIndex.get() + 1; i < currentScanIndex.get() + children.size(); i++)
            ((MeshView) children.get(i % children.size())).setMaterial(defaultMaterial);

         currentScanIndex.set((currentScanIndex.get() + 1) % numberOfScans.get());
      }
   }

   @Override
   public void unpackPointCloud(LidarScanMessage message)
   {
      if (message == null)
         return;

      Point3D32 scanPoint = new Point3D32();
      meshBuilder.clear();
      int numberOfScanPoints = message.getScan().size() / 3;
      for (int i = 0; i < numberOfScanPoints; i++)
      {
         double alpha = i / (double) numberOfScanPoints;
         Color color = Color.hsb(alpha * 240.0, 1.0, 1.0);

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
      return REAModuleAPI.UILidarScanShow;
   }

   @Override
   protected Topic<Boolean> createClearInput()
   {
      return REAModuleAPI.UILidarScanClear;
   }
}
