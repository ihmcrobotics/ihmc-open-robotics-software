package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
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
                                                                   staticRelativeSceneNode::freezeFromModification);
      distanceToDisableTrackingInput.setWidgetWidth(100.0f);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      ImGui.text("Current distance: %.2f".formatted(staticRelativeSceneNode.getCurrentDistance()));
      ImGui.sameLine();
      distanceToDisableTrackingInput.render();
   }

   @Override
   public StaticRelativeSceneNode getSceneNode()
   {
      return staticRelativeSceneNode;
   }
}
