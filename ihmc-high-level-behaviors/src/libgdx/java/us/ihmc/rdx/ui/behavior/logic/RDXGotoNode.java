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
      state.validateFields(rootNode.getOrderedLeaves());

      if (ImGui.beginCombo(labels.get("Goto"), definition.getNodeToGotoName()))
      {
         if (ImGui.selectable(labels.get("Next"), definition.getGotoNextNode()))
         {
            definition.setGotoNextNode();
         }

         for (LeafNodeState<?> leafNode : rootNode.getOrderedLeaves())
         {
            if (leafNode != state) // Exclude self
            {
               if (ImGui.selectable(leafNode.getDefinition().getName(), definition.getNodeToGotoID() == leafNode.getID()))
               {
                  definition.setNodeToGoto(leafNode.getID(), leafNode.getDefinition().getName());
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