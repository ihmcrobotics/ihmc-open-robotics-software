package us.ihmc.rdx.ui.behavior.logic;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.logic.GotoNodeDefinition;
import us.ihmc.behaviors.logic.GotoNodeState;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXLeafNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXGotoNode extends RDXLeafNode<GotoNodeState, GotoNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final GotoNodeState state;
   private final GotoNodeDefinition definition;

   public RDXGotoNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new GotoNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
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
   protected void renderImGuiWidgetsInternal()
   {
      BehaviorTreeRootNodeState rootNode = BehaviorTreeTools.findRootNode(state);

      // Validate state in case something earlier in this UI tick messed with things.
      // This happens with the Undo non-topological changes button.
      state.updateAndValidateGotoNode(rootNode.getOrderedLeaves());

      String selectedText;
      if (definition.getGotoNext().getValue())
      {
         selectedText = GotoNodeDefinition.GOTO_NEXT;
      }
      else
      {
         LeafNodeState<?> nodeToGoto = state.findNodeToGoto();
         selectedText = nodeToGoto.getDefinition().getName();
      }

      if (ImGui.beginCombo(labels.get("Goto"), selectedText))
      {
         if (ImGui.selectable(labels.get("Next"), definition.getGotoNext().getValue()))
         {
            definition.getGotoNext().setValue(true);
            definition.updateAndSanitizeGotoNodeFields(null);
         }

         for (LeafNodeState<?> leafNode : rootNode.getOrderedLeaves())
         {
            if (leafNode != state) // Exclude self
            {
               if (ImGui.selectable(leafNode.getDefinition().getName(), definition.getGotoNodeID().getValue() == leafNode.getID()))
               {
                  definition.getGotoNext().setValue(false);
                  definition.getGotoNodeID().setValue((int) leafNode.getID());
                  definition.updateAndSanitizeGotoNodeFields(leafNode.getDefinition().getName());
               }
            }
         }

         ImGui.endCombo();
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Goto Node";
   }
}