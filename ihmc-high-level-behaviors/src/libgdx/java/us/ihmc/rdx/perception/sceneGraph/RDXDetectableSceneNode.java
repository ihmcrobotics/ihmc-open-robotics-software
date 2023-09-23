package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphTreeModification;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;
import java.util.function.Consumer;

public class RDXDetectableSceneNode extends DetectableSceneNode implements RDXSceneNodeInterface
{
   private final RDXDetectableSceneNodeBasics detectableSceneNodeBasics;

   public RDXDetectableSceneNode(DetectableSceneNode nodeToCopy)
   {
      super(nodeToCopy.getID(), nodeToCopy.getName());

      detectableSceneNodeBasics = new RDXDetectableSceneNodeBasics(this);
   }

   @Override
   public void update(Consumer<SceneGraphTreeModification> modificationQueue)
   {
      detectableSceneNodeBasics.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      detectableSceneNodeBasics.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      detectableSceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
