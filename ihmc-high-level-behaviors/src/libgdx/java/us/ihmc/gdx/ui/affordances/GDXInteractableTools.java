package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;

public class GDXInteractableTools
{
   /**
    * This method creates a model that's slightly larger and half transparent
    * in order to serve as a "highlight" effect.
    *
    * Dispose with modelInstance.model.dispose().
    */
   public static ModelInstance createHighlightEffectModel(String modelFileName)
   {
      Model model = GDXModelLoader.loadG3DModel(modelFileName);
      ModelInstance modelInstance = new ModelInstance(model);
      modelInstance.transform.scale(1.01f, 1.01f, 1.01f);
      GDXTools.setTransparency(modelInstance, 0.5f);
      return modelInstance;
   }
}
