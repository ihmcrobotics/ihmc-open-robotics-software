package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

public class GDXRenderable implements RenderableProvider
{
   private final RenderableProvider renderableProvider;
   private final GDXSceneLevel sceneType;

   public GDXRenderable(RenderableProvider renderableProvider, GDXSceneLevel sceneType)
   {
      this.renderableProvider = renderableProvider;
      this.sceneType = sceneType;
   }

   public GDXSceneLevel getSceneType()
   {
      return sceneType;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      renderableProvider.getRenderables(renderables, pool);
   }
}
