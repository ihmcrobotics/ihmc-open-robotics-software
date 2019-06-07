package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.messager.Messager;

public class LidarImageFusionDataViewer
{
   private final Messager messager;

   private final AtomicReference<LidarImageFusionData> lidarImageFusionDataToRender = new AtomicReference<>(null);
   
   private final AtomicReference<MeshView> meshToRender = new AtomicReference<>(null);
   private final Group root = new Group();
   protected final ObservableList<Node> children = root.getChildren();
   private final AtomicReference<Boolean> clear = new AtomicReference<>(false);

   public LidarImageFusionDataViewer(SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
   }
   
   public void render()
   {
      MeshView newScanMeshView = meshToRender.getAndSet(null);

      if (clear.getAndSet(false))
         children.clear();

      if (newScanMeshView != null)
      {
         children.add(newScanMeshView);
      }
   }

   public void clear()
   {
      clear.set(true);
   }

   public Node getRoot()
   {
      return root;
   }
}
