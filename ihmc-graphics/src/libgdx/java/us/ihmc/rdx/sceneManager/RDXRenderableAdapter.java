package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import java.util.Set;

public class RDXRenderableAdapter implements RenderableProvider
{
   private final RDXRenderableProvider rdxRenderableProvider;
   private final RenderableProvider renderableProvider;
   private final RDXSceneLevel sceneLevel;
   private Set<RDXSceneLevel> sceneLevelsToRender;

   public RDXRenderableAdapter(RenderableProvider renderableProvider, RDXSceneLevel sceneLevel)
   {
      this.renderableProvider = renderableProvider;
      this.sceneLevel = sceneLevel;
      rdxRenderableProvider = null;
   }

   public RDXRenderableAdapter(RDXRenderableProvider rdxRenderableProvider)
   {
      this.rdxRenderableProvider = rdxRenderableProvider;
      renderableProvider = null;
      sceneLevel = null;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (rdxRenderableProvider != null)
      {
         rdxRenderableProvider.getRenderables(renderables, pool, sceneLevelsToRender);
      }
      else
      {
         if (sceneLevelsToRender.contains(this.sceneLevel))
         {
            renderableProvider.getRenderables(renderables, pool);
         }
      }
   }

   public void setSceneLevelsToRender(Set<RDXSceneLevel> sceneLevelsToRender)
   {
      this.sceneLevelsToRender = sceneLevelsToRender;
   }
}
