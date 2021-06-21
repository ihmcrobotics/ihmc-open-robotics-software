package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class GDXBoxVisualizer implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private ModelInstance modelInstance;
   private Model lastModel;
   private volatile Runnable toRender = null;

   private final Point3D[] vertices = new Point3D[8];
   private final Color color = new Color(0.7f, 0.7f, 0.7f, 1.0f);

   public GDXBoxVisualizer()
   {
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }
   }

   public void render()
   {
      if (toRender != null)
      {
         toRender.run();
         toRender = null;
      }
   }

   public void setColor(Color color)
   {
      this.color.set(color);
   }

   public void generateMeshAsync(Box3D box)
   {
      executorService.clearQueueAndExecute(() -> generateMesh(box));
   }

   public synchronized void generateMesh(Box3D box)
   {
      box.set(box);
      box.getVertices(vertices);

      toRender = () ->
      {
         if (lastModel != null)
            lastModel.dispose();

         lastModel = GDXModelPrimitives.buildModel(meshBuilder ->
         {
            double lineWidth = 0.03;
            meshBuilder.addMultiLineBox(vertices, lineWidth, color);
         }, "box");
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };
   }

   public void dispose()
   {
      executorService.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // sync over current and add
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }
}
