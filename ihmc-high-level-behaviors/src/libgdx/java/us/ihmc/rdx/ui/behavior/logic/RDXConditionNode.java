package us.ihmc.rdx.ui.behavior.logic;

import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImLongWrapper;
import us.ihmc.rdx.ui.behavior.sequence.RDXLeafNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXConditionNode extends RDXLeafNode<ConditionNodeState, ConditionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ConditionNodeDefinition definition;
   private final ConditionNodeState state;
   private final ImLongWrapper countWidget;
   private final ImLongWrapper countToWidget;

   public RDXConditionNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ConditionNodeState(id, crdtInfo, saveFileDirectory));

      definition = getDefinition();
      state = getState();

      countWidget = new ImLongWrapper(state.getCount()::getValue,
                                      state.getCount()::setValue,
                                      imLong -> ImGuiTools.volatileInputLong(labels.get("Count"), imLong));
      countToWidget = new ImLongWrapper(definition.getCountTo()::getValue,
                                        definition.getCountTo()::setValue,
                                        imLong -> ImGuiTools.volatileInputLong(labels.get("Count to"), imLong));
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
      countWidget.renderImGuiWidget();
      countToWidget.renderImGuiWidget();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Condition Node";
   }
}