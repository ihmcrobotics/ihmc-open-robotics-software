package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.rdx.ui.widgets.ImGuiSequenceIconWidget;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXActionSequence extends RDXBehaviorTreeNode<ActionSequenceState, ActionSequenceDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiSequenceIconWidget sequenceIconWidget = new ImGuiSequenceIconWidget();

   public RDXActionSequence(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ActionSequenceState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      if (sequenceIconWidget.render(!getChildren().isEmpty() && !getTreeWidgetExpanded()))
      {
         setSpecificWidgetOnRowClicked();
         setTreeWidgetExpanded(!getTreeWidgetExpanded());
      }
      ImGui.sameLine();
   }

   @Override
   public void renderContextMenuItems()
   {
      super.renderContextMenuItems();
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(definition.getClass().getSimpleName(), state.getID()));

      super.renderNodeSettingsWidgets();
   }
}