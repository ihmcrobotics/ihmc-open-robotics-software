package us.ihmc.footstepPlanning.ui.viewers;

import javafx.scene.Group;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import org.apache.commons.lang3.tuple.Pair;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Lightweight class for passing mesh data between threads
 * Set meshReference on any thread and call update() and remove() from an AnimationTimer handle method
 */
class MeshHolder
{
   final Group root;
   final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   final MeshView meshView = new MeshView();
   boolean addedFlag = false;

   MeshHolder(Group root)
   {
      this.root = root;
   }

   /**
    * Should be called from AnimationTimer.handle
    */
   void update()
   {
      Pair<Mesh, Material> mesh = meshReference.getAndSet(null);
      if (mesh != null)
      {
         if (!addedFlag)
         {
            root.getChildren().add(meshView);
            addedFlag = true;
         }

         meshView.setMesh(mesh.getKey());
         meshView.setMaterial(mesh.getValue());
      }
   }

   /**
    * Should be called from AnimationTimer.handle
    */
   void remove()
   {
      if (addedFlag)
      {
         root.getChildren().remove(meshView);
         addedFlag = false;
      }
   }
}
