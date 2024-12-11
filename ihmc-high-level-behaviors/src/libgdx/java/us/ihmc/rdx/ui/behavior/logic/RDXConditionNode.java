package us.ihmc.rdx.ui.behavior.logic;

import imgui.ImGui;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXConditionNode extends RDXBehaviorTreeNode<ConditionNodeState, ConditionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ConditionNodeState state;

   public RDXConditionNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ConditionNodeState(id, crdtInfo, saveFileDirectory));

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

      super.renderNodeSettingsWidgets();
   }
}