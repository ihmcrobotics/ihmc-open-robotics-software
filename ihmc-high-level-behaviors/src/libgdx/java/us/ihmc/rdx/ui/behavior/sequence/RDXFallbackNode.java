package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.FallbackNodeDefinition;
import us.ihmc.behaviors.sequence.FallbackNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXFallbackNode extends RDXBehaviorTreeNode<FallbackNodeState, FallbackNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final FallbackNodeState state;

   public RDXFallbackNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new FallbackNodeState(id, crdtInfo, saveFileDirectory));

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