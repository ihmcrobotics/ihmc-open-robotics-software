package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;

public interface RDXSceneNodeInterface
{
   default void update()
   {

   }

   default void renderImGuiWidgets()
   {
      
   }

   default void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {

   }
}
