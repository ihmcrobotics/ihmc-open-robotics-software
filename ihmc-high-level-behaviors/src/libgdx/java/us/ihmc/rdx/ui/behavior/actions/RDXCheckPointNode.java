package us.ihmc.rdx.ui.behavior.actions;

import us.ihmc.behaviors.sequence.actions.CheckPointNodeDefinition;
import us.ihmc.behaviors.sequence.actions.CheckPointNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXLeafNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXCheckPointNode extends RDXLeafNode<CheckPointNodeState, CheckPointNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXCheckPointNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new CheckPointNodeState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Check point";
   }
}