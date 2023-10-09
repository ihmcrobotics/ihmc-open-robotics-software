package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBodies.PredefinedRigidBodySceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.Set;

public class RDXPredefinedRigidBodySceneNode extends PredefinedRigidBodySceneNode implements RDXSceneNodeInterface
{
   private final RDXPredefinedRigidBodySceneNodeBasics predefinedRigidBodySceneNodeBasics;

   public RDXPredefinedRigidBodySceneNode(PredefinedRigidBodySceneNode nodeToCopy, RDX3DPanel panel3D)
   {
      super(nodeToCopy.getID(),
            nodeToCopy.getName(),
            nodeToCopy.getSceneGraphIDToNodeMap(),
            nodeToCopy.getInitialParentNodeID(),
            nodeToCopy.getInitialTransformToParent(),
            nodeToCopy.getVisualModelFilePath(),
            nodeToCopy.getVisualModelToNodeFrameTransform());

      predefinedRigidBodySceneNodeBasics = new RDXPredefinedRigidBodySceneNodeBasics(this, panel3D);
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      predefinedRigidBodySceneNodeBasics.update(modificationQueue);
   }

   @Override
   public void renderImGuiWidgets()
   {
      predefinedRigidBodySceneNodeBasics.renderImGuiWidgets();
   }

   @Override
   public void renderRemove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      predefinedRigidBodySceneNodeBasics.renderRemove(modificationQueue, sceneGraph);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      predefinedRigidBodySceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
