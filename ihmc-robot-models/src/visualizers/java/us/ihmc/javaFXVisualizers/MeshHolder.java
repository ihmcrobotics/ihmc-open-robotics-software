package us.ihmc.javaFXVisualizers;

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
public class MeshHolder
{
   private final Group root;
   private final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   private final MeshView meshView = new MeshView();
   private boolean addedFlag = false;

   public MeshHolder(Group root)
   {
      this.root = root;
   }
   
   public void setMeshReference(Pair<Mesh, Material> reference)
   {
      meshReference.set(reference);
   }
   
   public MeshView getMeshView()
   {
      return meshView;
   }

   /**
    * Should be called from AnimationTimer.handle
    */
   public void update()
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
   public void remove()
   {
      if (addedFlag)
      {
         root.getChildren().remove(meshView);
         addedFlag = false;
      }
   }
}
