package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.gdx.tools.GDXModelPrimitives;

public class GDXVRTeleporter implements RenderableProvider
{
   private boolean teleporting = false;

   private ModelInstance line;
   private ModelInstance ring;

   public void create()
   {
      line = GDXModelPrimitives.buildModelInstance(meshBuilder ->
      {
         meshBuilder.addLine(0.0, 0.0, 0.0, 5.0, 5.0, 5.0, 0.003, Color.BLUE);
      }, "line");

      ring = GDXModelPrimitives.buildModelInstance(meshBuilder ->
      {
//         meshBuilder.add
      }, "ring");
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (teleporting)
      {
         // render stuff
      }
   }
}
