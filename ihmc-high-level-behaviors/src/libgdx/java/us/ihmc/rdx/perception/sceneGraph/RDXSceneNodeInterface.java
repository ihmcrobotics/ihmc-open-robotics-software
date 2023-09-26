package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;

public interface RDXSceneNodeInterface
{
   /**
    * Updates the UI node.
    * This is tricky. Modifications can't be done in here because
    * that would be a concurrent modification of the tree while iterating
    * over it, so the entire UI node
    * tree update is wrapped in {@link SceneGraph#modifyTree}.
    */
   default void update(SceneGraphModificationQueue modificationQueue)
   {

   }

   default void renderImGuiWidgets()
   {
      
   }

   default void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {

   }
}
