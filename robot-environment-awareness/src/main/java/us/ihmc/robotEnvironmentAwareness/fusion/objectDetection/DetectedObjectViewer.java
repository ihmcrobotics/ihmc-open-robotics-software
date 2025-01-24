package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.MeshView;
import us.ihmc.ros2.ROS2Node;

import java.util.concurrent.atomic.AtomicReference;

public class DetectedObjectViewer
{
   private final Group root = new Group();
   protected final ObservableList<Node> children = root.getChildren();

   private final AtomicReference<MeshView> meshToRender = new AtomicReference<>(null);

   private final AtomicReference<Boolean> clear = new AtomicReference<>(false);

   public DetectedObjectViewer(ROS2Node ros2Node)
   {
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
