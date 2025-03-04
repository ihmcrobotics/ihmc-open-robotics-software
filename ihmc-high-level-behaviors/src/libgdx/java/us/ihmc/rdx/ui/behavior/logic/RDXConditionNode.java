package us.ihmc.rdx.ui.behavior.logic;

import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.logic.condition.RDXCounterCondition;
import us.ihmc.rdx.ui.behavior.sequence.RDXLeafNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXConditionNode extends RDXLeafNode<ConditionNodeState, ConditionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private RDXCounterCondition counter;

   public RDXConditionNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ConditionNodeState(id, crdtInfo, saveFileDirectory));


   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      switch (definition.getType().getValue())
      {
         case COUNTER -> counter.renderImGuiWidgetsInternal();
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Condition Node";
   }
}