package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.FallbackNodeDefinition;
import us.ihmc.behaviors.sequence.FallbackNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXFallbackNode extends RDXBehaviorTreeNode<FallbackNodeState, FallbackNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final FallbackNodeDefinition definition;
   private final FallbackNodeState state;

   public RDXFallbackNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new FallbackNodeState(id, crdtInfo, saveFileDirectory));

      definition = getDefinition();
      state = getState();
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void renderContextMenuItems()
   {
      super.renderContextMenuItems();
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(getDefinition().getClass().getSimpleName(), getState().getID()));

      BehaviorTreeRootNodeState actionSequence = BehaviorTreeTools.findRootNode(state);

      if (actionSequence != null)
      {
         String selectedText = definition.getGotoActionName();

         if (selectedText == null)
            selectedText = "Not specified";

         if (ImGui.beginCombo(labels.get("Go to"), selectedText))
         {
            if (ImGui.selectable(labels.get("Not specified"), definition.getGotoActionID().getValue() == 0))
            {
               definition.setGotoActionName(null);
               definition.getGotoActionID().setValue(0);
            }

            for (ActionNodeState<?> actionChild : actionSequence.getActionChildren())
            {
               if (ImGui.selectable(labels.get(actionChild.getDefinition().getName()), definition.getGotoActionID().getValue() == actionChild.getID()))
               {
                  definition.setGotoActionName(actionChild.getDefinition().getName());
                  definition.getGotoActionID().setValue(actionChild.getID());
               }
            }

            ImGui.endCombo();
         }
      }

      super.renderNodeSettingsWidgets();
   }
}