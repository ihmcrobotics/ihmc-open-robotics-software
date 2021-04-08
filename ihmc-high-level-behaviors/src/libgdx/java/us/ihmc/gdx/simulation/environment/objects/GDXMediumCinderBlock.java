package us.ihmc.gdx.simulation.environment.objects;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXMediumCinderBlock
{
   private ModelInstance modelInstance;

   public void create(GDX3DSceneManager sceneManager)
   {
      modelInstance = new ModelInstance(GDXModelLoader.loadG3DModel("mediumCinderBlock.g3dj"));
      sceneManager.addModelInstance(modelInstance);
   }
}
