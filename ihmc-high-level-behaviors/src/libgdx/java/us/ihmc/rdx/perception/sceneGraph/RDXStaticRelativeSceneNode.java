package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.rdx.imgui.ImGuiInputDoubleWrapper;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.Set;

public class RDXStaticRelativeSceneNode extends StaticRelativeSceneNode implements RDXSceneNodeInterface
{
   private final RDXPredefinedRigidBodySceneNodeBasics predefinedRigidBodySceneNodeBasics;
   private final ImGuiInputDoubleWrapper distanceToDisableTrackingInput;

   public RDXStaticRelativeSceneNode(StaticRelativeSceneNode nodeToCopy, RDX3DPanel panel3D)
   {
      super(nodeToCopy.getID(),
            nodeToCopy.getName(),
            nodeToCopy.getSceneGraphIDToNodeMap(),
            nodeToCopy.getInitialParentNodeID(),
            nodeToCopy.getInitialTransformToParent(),
            nodeToCopy.getVisualModelFilePath(),
            nodeToCopy.getVisualModelToNodeFrameTransform(),
            nodeToCopy.getDistanceToDisableTracking());

      predefinedRigidBodySceneNodeBasics = new RDXPredefinedRigidBodySceneNodeBasics(this, panel3D);

      distanceToDisableTrackingInput = new ImGuiInputDoubleWrapper("Distance to disable tracking", "%.2f", 0.1, 0.5,
                                                                   this::getDistanceToDisableTracking,
                                                                   this::setDistanceToDisableTracking,
                                                                   this::freezeFromModification);
      distanceToDisableTrackingInput.setWidgetWidth(100.0f);
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

      ImGui.text("Current distance: %.2f".formatted(getCurrentDistance()));
      ImGui.sameLine();
      distanceToDisableTrackingInput.render();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      predefinedRigidBodySceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
