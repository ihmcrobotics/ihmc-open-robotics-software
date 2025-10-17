package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.CheckPointNodeDefinition;
import us.ihmc.behaviors.sequence.actions.CheckPointNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.behavior.sequence.RDXLeafNode;
import us.ihmc.rdx.ui.widgets.ImGuiCheckpointNodeWidget;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXCheckPointNode extends RDXLeafNode<CheckPointNodeState, CheckPointNodeDefinition>
{
   private final ImGuiCheckpointNodeWidget checkpointNodeWidget = new ImGuiCheckpointNodeWidget();

   public RDXCheckPointNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new CheckPointNodeState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      checkpointNodeWidget.render();
      ImGui.sameLine();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Check point";
   }
}