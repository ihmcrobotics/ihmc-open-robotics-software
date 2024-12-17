package us.ihmc.rdx.ui.behavior.actions;

import us.ihmc.behaviors.sequence.actions.CheckPointNodeDefinition;
import us.ihmc.behaviors.sequence.actions.CheckPointNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXCheckPointNode extends RDXActionNode<CheckPointNodeState, CheckPointNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CheckPointNodeState state;

   public RDXCheckPointNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new CheckPointNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
   }

   @Override
   public String getActionTypeTitle()
   {
      return String.format("Check point");
   }
}