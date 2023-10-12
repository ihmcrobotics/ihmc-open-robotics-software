package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.Set;

public class RDXPredefinedRigidBodySceneNode extends RDXRigidBodySceneNode
{
   private static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);

   private final RDXModelInstance modelInstance;
   private transient final RigidBodyTransform visualModelToWorldTransform = new RigidBodyTransform();

   public RDXPredefinedRigidBodySceneNode(PredefinedRigidBodySceneNode predefinedRigidBodySceneNode, RDX3DPanel panel3D)
   {
      super(predefinedRigidBodySceneNode, panel3D);

      modelInstance = new RDXModelInstance(RDXModelLoader.load(predefinedRigidBodySceneNode.getVisualModelFilePath()));
      modelInstance.setColor(GHOST_COLOR);
   }

   public void update(SceneGraphModificationQueue modificationQueue)
   {
      super.update(modificationQueue);
      nodePose.get(visualModelToWorldTransform);
      modelInstance.setTransformToWorldFrame(visualModelToWorldTransform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      if (sceneLevels.contains(RDXSceneLevel.MODEL))
         modelInstance.getRenderables(renderables, pool);
   }
}
