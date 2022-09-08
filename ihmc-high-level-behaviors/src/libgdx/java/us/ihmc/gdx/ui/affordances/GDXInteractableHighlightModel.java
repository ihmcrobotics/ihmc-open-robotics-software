package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.interactable.GDXModelInstanceScaler;

/**
 * This class creates a model that's slightly larger and half transparent
 * in order to serve as a "highlight" effect.
 */
public class GDXInteractableHighlightModel implements RenderableProvider
{
   private final ModelInstance modelInstance;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final GDXModelInstanceScaler scaledModelInstance;

   public GDXInteractableHighlightModel(String modelFileName)
   {
      this(GDXModelLoader.load(modelFileName));
   }

   public GDXInteractableHighlightModel(Model model)
   {
      modelInstance = new ModelInstance(model);
      scaledModelInstance = new GDXModelInstanceScaler(modelInstance);
      double scaleFactor = 1.01;
      scaledModelInstance.scale(scaleFactor);
      setTransparency(0.5);
   }

   public void setPose(RigidBodyTransform transformToWorld, RigidBodyTransform additionalTransform)
   {
      tempTransform.set(additionalTransform);
      transformToWorld.transform(tempTransform);
      setPose(tempTransform);
   }

   public void setPose(RigidBodyTransform transformToWorld)
   {
      GDXTools.toGDX(transformToWorld, scaledModelInstance.getPoseTransform());
   }

   public void setTransparency(double transparency)
   {
      GDXTools.setTransparency(modelInstance, (float) transparency);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      modelInstance.model.dispose();
   }
}
