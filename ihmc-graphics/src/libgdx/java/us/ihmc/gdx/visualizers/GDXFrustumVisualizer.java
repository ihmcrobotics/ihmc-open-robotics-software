package us.ihmc.gdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Frustum;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class GDXFrustumVisualizer implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private ModelInstance modelInstance;
   private Model lastModel;
   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private final Point3D[] vertices = new Point3D[16];
   private final Color color = new Color(0.7f, 0.7f, 0.7f, 1.0f);

   public GDXFrustumVisualizer()
   {
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }
   }

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void setColor(Color color)
   {
      this.color.set(color);
   }

   public void generateMeshAsync(Frustum frustum)
   {
      executorService.clearQueueAndExecute(() -> generateMesh(frustum));
   }

   public synchronized void generateMesh(Frustum frustum)
   {
      GDXTools.toEuclid(frustum.planePoints[0], vertices[0]);
      GDXTools.toEuclid(frustum.planePoints[1], vertices[1]);
      GDXTools.toEuclid(frustum.planePoints[2], vertices[2]);
      GDXTools.toEuclid(frustum.planePoints[3], vertices[3]);
      GDXTools.toEuclid(frustum.planePoints[0], vertices[4]);
      GDXTools.toEuclid(frustum.planePoints[4], vertices[5]);
      GDXTools.toEuclid(frustum.planePoints[7], vertices[6]);
      GDXTools.toEuclid(frustum.planePoints[3], vertices[7]);
      GDXTools.toEuclid(frustum.planePoints[7], vertices[8]);
      GDXTools.toEuclid(frustum.planePoints[6], vertices[9]);
      GDXTools.toEuclid(frustum.planePoints[2], vertices[10]);
      GDXTools.toEuclid(frustum.planePoints[6], vertices[11]);
      GDXTools.toEuclid(frustum.planePoints[5], vertices[12]);
      GDXTools.toEuclid(frustum.planePoints[1], vertices[13]);
      GDXTools.toEuclid(frustum.planePoints[5], vertices[14]);
      GDXTools.toEuclid(frustum.planePoints[4], vertices[15]);

      buildMeshAndCreateModelInstance = () ->
      {
         if (lastModel != null)
            lastModel.dispose();

         lastModel = GDXModelBuilder.buildModel(meshBuilder ->
         {
            double lineWidth = 0.01;
            meshBuilder.addMultiLine(vertices, lineWidth, color, false);
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
