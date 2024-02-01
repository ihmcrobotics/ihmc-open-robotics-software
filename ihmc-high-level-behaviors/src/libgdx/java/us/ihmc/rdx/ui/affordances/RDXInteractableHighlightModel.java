package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstanceScaler;

/**
 * This class creates a model that's slightly larger and half transparent
 * in order to serve as a "highlight" effect.
 */
public class RDXInteractableHighlightModel implements RenderableProvider
{
   private final RDXModelInstanceScaler scaledModelInstance;

   public RDXInteractableHighlightModel(String modelFileName)
   {
      this(RDXModelLoader.loadModelData(modelFileName));
   }

   public RDXInteractableHighlightModel(ModelData modelData)
   {
      double startingScaleFactor = 1.01;
      scaledModelInstance = new RDXModelInstanceScaler(modelData);
      scaledModelInstance.scale(startingScaleFactor);
      setTransparency(0.5);
   }

   public void setPose(ReferenceFrame referenceFrame)
   {
      setPose(referenceFrame.getTransformToRoot());
   }

   public void setPose(RigidBodyTransform transformToWorld)
   {
      LibGDXTools.toLibGDX(transformToWorld, scaledModelInstance.getPoseTransform());
   }

   public void setTransparency(double transparency)
   {
      LibGDXTools.setOpacity(scaledModelInstance.getModelInstance(), (float) transparency);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      scaledModelInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      scaledModelInstance.getModelInstance().model.dispose();
   }
}
