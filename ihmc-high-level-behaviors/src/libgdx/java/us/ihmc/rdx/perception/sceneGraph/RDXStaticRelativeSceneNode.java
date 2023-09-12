package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.TLongObjectMap;
import imgui.ImGui;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.rdx.imgui.ImGuiInputDoubleWrapper;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.Set;

public class RDXStaticRelativeSceneNode extends StaticRelativeSceneNode implements RDXSceneNodeInterface
{
   private final RDXPredefinedRigidBodySceneNodeBasics predefinedRigidBodySceneNodeBasics;
   private final ImGuiInputDoubleWrapper distanceToDisableTrackingInput;

   public RDXStaticRelativeSceneNode(long id,
                                     String name,
                                     TLongObjectMap<SceneNode> sceneGraphIDToNodeMap,
                                     long initialParentNodeID,
                                     RigidBodyTransformReadOnly initialTransformToParent,
                                     String visualModelFilePath,
                                     RigidBodyTransform visualModelToNodeFrameTransform,
                                     double distanceToDisableTracking,
                                     RDX3DPanel panel3D)
   {
      super(id,
            name,
            sceneGraphIDToNodeMap,
            initialParentNodeID,
            initialTransformToParent,
            visualModelFilePath,
            visualModelToNodeFrameTransform,
            distanceToDisableTracking);

      predefinedRigidBodySceneNodeBasics = new RDXPredefinedRigidBodySceneNodeBasics(this, panel3D);

      distanceToDisableTrackingInput = new ImGuiInputDoubleWrapper("Distance to disable tracking", "%.2f", 0.1, 0.5,
                                                                   this::getDistanceToDisableTracking,
                                                                   this::setDistanceToDisableTracking,
                                                                   this::markModifiedByOperator);
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

      ImGui.text("Current distance: %.2f".formatted(getCurrentDistance()));
      distanceToDisableTrackingInput.render();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      predefinedRigidBodySceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
