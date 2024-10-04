package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import java.util.Set;

/**
 * This interface provides a set of the currently active scene levels so things
 * can dynamically decide what scene levels to place renderables in and what
 * things get rendered in them.
 */
public interface RDXRenderableProvider
{
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels);
}
