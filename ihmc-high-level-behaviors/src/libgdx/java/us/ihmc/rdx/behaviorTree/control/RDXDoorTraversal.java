package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalDefinition;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXDoorTraversal extends RDXBehaviorTreeNode<DoorTraversalState, DoorTraversalDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXDoorTraversal(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new DoorTraversalState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(definition.getClass().getSimpleName(), state.getID()));


      super.renderNodeSettingsWidgets();
   }
}