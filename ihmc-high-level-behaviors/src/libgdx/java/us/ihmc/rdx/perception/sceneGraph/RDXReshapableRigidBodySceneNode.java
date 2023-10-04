package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBodies.ReshapableRigidBodySceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.Set;

public class RDXReshapableRigidBodySceneNode extends ReshapableRigidBodySceneNode implements RDXSceneNodeInterface
{
   private final RDXReshapableRigidBodySceneNodeBasics reshapableRigidBodySceneNodeBasics;

   public RDXReshapableRigidBodySceneNode(ReshapableRigidBodySceneNode nodeToCopy, RDX3DPanel panel3D)
   {
      super(nodeToCopy.getID(),
            nodeToCopy.getName(),
            nodeToCopy.getSceneGraphIDToNodeMap(),
            nodeToCopy.getInitialParentNodeID(),
            nodeToCopy.getInitialTransformToParent());

      reshapableRigidBodySceneNodeBasics = new RDXReshapableRigidBodySceneNodeBasics(this, panel3D);
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      reshapableRigidBodySceneNodeBasics.update(modificationQueue);
   }

   @Override
   public void renderImGuiWidgets()
   {
      reshapableRigidBodySceneNodeBasics.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      reshapableRigidBodySceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
