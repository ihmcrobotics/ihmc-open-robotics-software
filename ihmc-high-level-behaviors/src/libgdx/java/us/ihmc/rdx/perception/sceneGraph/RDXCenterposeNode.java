package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;

public class RDXCenterposeNode extends CenterposeNode implements RDXSceneNodeInterface
{
   public RDXCenterposeNode(long id, String name)
   {
      super(id, name);
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
//      RDXSceneNodeInterface.super.update(modificationQueue);
   }

   @Override
   public void renderImGuiWidgets()
   {
//      RDXSceneNodeInterface.super.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
//      RDXSceneNodeInterface.super.getRenderables(renderables, pool, sceneLevels);
   }
}
