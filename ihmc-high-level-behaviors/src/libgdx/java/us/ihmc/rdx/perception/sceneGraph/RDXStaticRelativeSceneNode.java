package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.rdx.imgui.ImGuiInputDoubleWrapper;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXStaticRelativeSceneNode extends RDXPredefinedRigidBodySceneNode
{
   private final StaticRelativeSceneNode staticRelativeSceneNode;
   private final ImGuiInputDoubleWrapper distanceToDisableTrackingInput;

   public RDXStaticRelativeSceneNode(StaticRelativeSceneNode staticRelativeSceneNode, RDX3DPanel panel3D)
   {
      super(staticRelativeSceneNode, panel3D);
      this.staticRelativeSceneNode = staticRelativeSceneNode;

      distanceToDisableTrackingInput = new ImGuiInputDoubleWrapper("Distance to disable tracking",
                                                                   "%.2f",
                                                                   0.1,
                                                                   0.5,
                                                                   staticRelativeSceneNode::getDistanceToDisableTracking,
                                                                   staticRelativeSceneNode::setDistanceToDisableTracking,
                                                                   staticRelativeSceneNode::freeze);
      distanceToDisableTrackingInput.setWidgetWidth(100.0f);
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      ImGui.text("Current distance: %.2f".formatted(staticRelativeSceneNode.getCurrentDistance()));
      ImGui.sameLine();
      distanceToDisableTrackingInput.renderImGuiWidget();
   }

   @Override
   public StaticRelativeSceneNode getSceneNode()
   {
      return staticRelativeSceneNode;
   }
}
