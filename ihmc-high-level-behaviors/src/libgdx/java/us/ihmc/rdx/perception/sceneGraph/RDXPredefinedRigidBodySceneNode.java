package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.TLongObjectMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBodies.PredefinedRigidBodySceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.Set;

public class RDXPredefinedRigidBodySceneNode extends PredefinedRigidBodySceneNode implements RDXSceneNodeInterface
{
   private final RDXPredefinedRigidBodySceneNodeBasics predefinedRigidBodySceneNodeBasics;

   public RDXPredefinedRigidBodySceneNode(long id,
                                          String name,
                                          TLongObjectMap<SceneNode> sceneGraphIDToNodeMap,
                                          long initialParentNodeID,
                                          RigidBodyTransformReadOnly initialTransformToParent,
                                          String visualModelFilePath,
                                          RigidBodyTransform visualModelToNodeFrameTransform,
                                          RDX3DPanel panel3D)
   {
      super(id, name, sceneGraphIDToNodeMap, initialParentNodeID, initialTransformToParent, visualModelFilePath, visualModelToNodeFrameTransform);
      predefinedRigidBodySceneNodeBasics = new RDXPredefinedRigidBodySceneNodeBasics(this, panel3D);
   }

   @Override
   public void update()
   {
      predefinedRigidBodySceneNodeBasics.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      predefinedRigidBodySceneNodeBasics.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      predefinedRigidBodySceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
