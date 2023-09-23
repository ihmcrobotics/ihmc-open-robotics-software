package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeMove;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.List;
import java.util.Set;

/**
 * A reference frame graphic in the virtual scene
 * to be used for debugging and know where the frame is.
 *
 * We choose to have the UI nodes extend the underlying scene nodes
 * in order to avoid complicated sync logic. This means though,
 * that we cannot have UI node types extend each other, because it
 * would require multiple inheritance. So instead we create some basics
 * classes to share common functionality and an interface.
 */
public class RDXSceneNode extends SceneNode implements RDXSceneNodeInterface
{
   private final RDXSceneNodeBasics sceneNodeBasics;

   public RDXSceneNode(long id, String name)
   {
      super(id, name);
      sceneNodeBasics = new RDXSceneNodeBasics(this);
   }

   @Override
   public void update(List<SceneGraphNodeMove> sceneGraphNodeMoves)
   {
      sceneNodeBasics.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      sceneNodeBasics.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      sceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
