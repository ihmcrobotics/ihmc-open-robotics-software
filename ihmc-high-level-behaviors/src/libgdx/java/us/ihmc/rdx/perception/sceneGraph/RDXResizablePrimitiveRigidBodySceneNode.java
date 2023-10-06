package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBodies.PrimitiveRigidBodySceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.Set;

public class RDXResizablePrimitiveRigidBodySceneNode extends PrimitiveRigidBodySceneNode implements RDXSceneNodeInterface
{
   private final RDXResizablePrimitiveRigidBodySceneNodeBasics resizableRigidBodySceneNodeBasics;

   public RDXResizablePrimitiveRigidBodySceneNode(PrimitiveRigidBodySceneNode nodeToCopy, RDX3DPanel panel3D)
   {
      super(nodeToCopy.getID(),
            nodeToCopy.getName(),
            nodeToCopy.getSceneGraphIDToNodeMap(),
            nodeToCopy.getInitialParentNodeID(),
            nodeToCopy.getInitialTransformToParent(),
            nodeToCopy.getShape());

      resizableRigidBodySceneNodeBasics = new RDXResizablePrimitiveRigidBodySceneNodeBasics(this, panel3D);
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      resizableRigidBodySceneNodeBasics.update(modificationQueue);
   }

   @Override
   public void renderImGuiWidgets()
   {
      resizableRigidBodySceneNodeBasics.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      resizableRigidBodySceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
