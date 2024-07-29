package us.ihmc.rdx.simulation.environment;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;

public class RDXModelHighlighting
{
   private Material originalMaterial;
   private Material highlightedMaterial;

   public void setHighlighted(ModelInstance modelInstance, boolean highlighted)
   {
      if (originalMaterial == null)
      {
         originalMaterial = new Material(modelInstance.materials.get(0));
      }

      if (highlighted)
      {
         if (highlightedMaterial == null)
         {
            highlightedMaterial = new Material();
            highlightedMaterial.set(PBRColorAttribute.createBaseColorFactor(Color.ORANGE));
         }

         modelInstance.materials.get(0).set(highlightedMaterial);
      }
      else
      {
         modelInstance.materials.get(0).set(originalMaterial);
      }
   }
}
