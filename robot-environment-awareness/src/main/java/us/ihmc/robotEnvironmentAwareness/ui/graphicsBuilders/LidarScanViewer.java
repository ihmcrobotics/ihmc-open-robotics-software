package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

/**
 * Created by adrien on 11/20/16.
 */
public class LidarScanViewer implements Runnable
{
   private static final float SCAN_POINT_SIZE = 0.0075f;
   private static final Material defaultMaterial = new PhongMaterial(Color.DARKRED);

   private final Group root = new Group();
   private final ObservableList<Node> children = root.getChildren();

   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(128));

   private final AtomicReference<LidarScanMessage> newMessageToRender;
   private final AtomicReference<MeshView> scanMeshToRender = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;
   private final AtomicReference<Integer> numberOfScans;
   private final AtomicInteger currentScanIndex = new AtomicInteger(0);

   private final REAUIMessager uiMessager;

   public LidarScanViewer(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      newMessageToRender = uiMessager.createInput(REAModuleAPI.LidarScanState);
      enable = uiMessager.createInput(REAModuleAPI.UILidarScanShow, false);
      clear = uiMessager.createInput(REAModuleAPI.UILidarScanClear, false);
      numberOfScans = uiMessager.createInput(REAModuleAPI.UILidarScanSize, 50);
   }

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
   public void run()
   {
      if (!enable.get())
         return;

      LidarScanMessage message = newMessageToRender.getAndSet(null);

      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestLidarScan);

      if (message == null)
         return;

      Point3D32 scanPoint = new Point3D32();
      meshBuilder.clear();
      for (int i = 0; i < message.getNumberOfScanPoints(); i++)
      {
         double alpha = i / (double) message.getNumberOfScanPoints();
         Color color = Color.hsb(alpha * 240.0, 1.0, 1.0);

         message.getScanPoint(i, scanPoint);

         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, color);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      scanMeshToRender.set(scanMeshView);
      meshBuilder.clear();
   }

   public Node getRoot()
   {
      return root;
   }
}
