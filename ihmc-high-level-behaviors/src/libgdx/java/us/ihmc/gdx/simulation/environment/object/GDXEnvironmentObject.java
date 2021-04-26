package us.ihmc.gdx.simulation.environment.object;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;

public class GDXEnvironmentObject
{
   protected final Model model;
   private final Pose3D pose = new Pose3D();

   private final GDXModelInstance modelInstance;
   private final Material originalMaterial;
   private Material highlightedMaterial;

   public GDXEnvironmentObject(Model model)
   {
      this.model = model;
      modelInstance = new GDXModelInstance(model);
      originalMaterial = new Material(modelInstance.materials.get(0));
   }

   public void setHighlighted(boolean highlighted)
   {
      if (highlighted)
      {
         if (highlightedMaterial == null)
         {
            highlightedMaterial = new Material();
            highlightedMaterial.set(ColorAttribute.createDiffuse(Color.ORANGE));
         }

         modelInstance.materials.get(0).set(highlightedMaterial);
      }
      else
      {
         modelInstance.materials.get(0).set(originalMaterial);
      }
   }

   public GDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public GDXEnvironmentObject duplicate()
   {
      return new GDXEnvironmentObject(model);
   }
}
