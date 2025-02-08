package us.ihmc.rdx.ui.behavior.logic;

import imgui.ImGui;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXLeafNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXConditionNode extends RDXLeafNode<ConditionNodeState, ConditionNodeDefinition>
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
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.text("Count: %d".formatted(state.getCount().getValue()));

      ImGuiTools.volatileInputLong(labels.get("Count to"), )

      long[] countTo = {definition.getCountTo().getValue()};
      if (ImGui.sliderLong("Set Count To", countTo, 0, Long.MAX_VALUE))
      {
         definition.getCountTo().setValue(countTo[0]);
      }
       
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Condition Node";
   }
}