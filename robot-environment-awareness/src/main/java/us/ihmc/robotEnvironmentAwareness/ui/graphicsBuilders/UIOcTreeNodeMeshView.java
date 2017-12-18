package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.jOctoMap.key.OcTreeKey;

public class UIOcTreeNodeMeshView extends MeshView
{
   private final OcTreeKey nodeKey;

   public UIOcTreeNodeMeshView(OcTreeKey nodeKey)
   {
      this.nodeKey = nodeKey;
   }

   public UIOcTreeNodeMeshView(OcTreeKey nodeKey, Mesh mesh, Material material)
   {
      super(mesh);
      this.nodeKey = nodeKey;
      setMaterial(material);
   }

   @Override
   public int hashCode()
   {
      return nodeKey.hashCode();
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj instanceof UIOcTreeNodeMeshView)
         return equals((UIOcTreeNodeMeshView) obj);
      else
         return false;
   }

   public boolean equals(UIOcTreeNodeMeshView other)
   {
      return nodeKey.equals(other.nodeKey);
   }
}
