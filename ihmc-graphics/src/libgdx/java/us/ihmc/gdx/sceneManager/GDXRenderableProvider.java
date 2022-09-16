package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import java.util.Set;

public interface GDXRenderableProvider
{
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<GDXSceneLevel> sceneLevels);
}
