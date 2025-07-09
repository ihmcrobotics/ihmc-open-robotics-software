package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

/**
 * This class works with a copy of ModelData that it will mutate to create scaled ModelInstances.
 * Every time scale is called, the modelIntance will be recreated as a new object.
 *
 * The scaling is about the center of mass of the vertices, i.e. the centroid. This is so the model will
 * get bigger but not translate much, so that scaled up models will be in the same place roughly.
 * This is useful when you want a bigger model to enclose a smaller version.
 *
 * TODO: Dynamically scale using the shader or scale in parallel on the GPU somehow.
 *   Usually, the scale is represented as a uniform and sent to the shader to be used for changing scale before rendering.
 *   i.e. how OpenGL changes point cloud point size for spheres/points. It never modifies the actual vertex data on the CPU.
 */
public class RDXModelInstanceScaler
{
   private final Model model;
   private RDXModelInstance modelInstance;

   public RDXModelInstanceScaler(String modelFileName)
   {
      this(RDXModelLoader.load(modelFileName));
   }

   public RDXModelInstanceScaler(Model model)
   {
      // Deep copy the model so we can modify it without affecting the original model,
      // which we only load one of each to save resources.
      this.model = LibGDXModelCopier.deepCopy(model);

      modelInstance = new RDXModelInstance(model);
   }

   private void scaleInternal(Model model, double scale)
   {
      for (int i = 0; i < model.nodes.size; i++)
         model.nodes.get(i).scale.set((float) scale, (float) scale, (float) scale);
   }

   public void scale(double scale)
   {
      scaleInternal(model, scale);
      modelInstance = new RDXModelInstance(model);
   }

   /**
    * First deep copies the held model, then scales and returns it,
    * so nothing else is affected.
    */
   public Model getScaledDeepCopy(double scale)
   {
      Model copiedModel = LibGDXModelCopier.deepCopy(model);
      scaleInternal(copiedModel, scale);
      return copiedModel;
   }

   public Matrix4 getPoseTransform()
   {
      return modelInstance.transform;
   }

   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }
}
