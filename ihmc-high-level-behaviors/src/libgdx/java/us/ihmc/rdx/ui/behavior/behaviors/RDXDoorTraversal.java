package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.door.DoorTraversalState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXDoorTraversal extends RDXBehaviorTreeNode<DoorTraversalState, DoorTraversalDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DoorTraversalState state;

   public RDXDoorTraversal(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();

      getDefinition().setName("Door traversal");
   }

   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);
   }

   public void updateActionSubtree(RDXBehaviorTreeNode<?, ?> node)
   {
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXActionNode<?, ?> actionNode)
         {

         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(getDefinition().getClass().getSimpleName(), getState().getID()));

      super.renderNodeSettingsWidgets();
   }
}