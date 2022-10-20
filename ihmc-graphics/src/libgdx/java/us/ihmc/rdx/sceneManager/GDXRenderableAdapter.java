package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import java.util.Set;

public class GDXRenderableAdapter implements RenderableProvider
{
   private final GDXRenderableProvider gdxRenderableProvider;
   private final RenderableProvider renderableProvider;
   private final GDXSceneLevel sceneLevel;
   private Set<GDXSceneLevel> sceneLevelsToRender;

   public GDXRenderableAdapter(RenderableProvider renderableProvider, GDXSceneLevel sceneLevel)
   {
      this.renderableProvider = renderableProvider;
      this.sceneLevel = sceneLevel;
      gdxRenderableProvider = null;
   }

   public GDXRenderableAdapter(GDXRenderableProvider gdxRenderableProvider)
   {
      this.gdxRenderableProvider = gdxRenderableProvider;
      renderableProvider = null;
      sceneLevel = null;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (gdxRenderableProvider != null)
      {
         gdxRenderableProvider.getRenderables(renderables, pool, sceneLevelsToRender);
      }
      else
      {
         if (sceneLevelsToRender.contains(this.sceneLevel))
         {
            renderableProvider.getRenderables(renderables, pool);
         }
      }
   }

   public void setSceneLevelsToRender(Set<GDXSceneLevel> sceneLevelsToRender)
   {
      this.sceneLevelsToRender = sceneLevelsToRender;
   }
}
