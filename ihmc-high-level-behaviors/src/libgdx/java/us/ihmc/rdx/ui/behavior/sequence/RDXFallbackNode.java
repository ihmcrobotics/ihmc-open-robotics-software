package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.FallbackNodeDefinition;
import us.ihmc.behaviors.sequence.FallbackNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.rdx.ui.widgets.ImGuiFallbackWidget;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXFallbackNode extends RDXBehaviorTreeNode<FallbackNodeState, FallbackNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFallbackWidget fallbackWidget = new ImGuiFallbackWidget();

   public RDXFallbackNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new FallbackNodeState(id, crdtInfo, saveFileDirectory));
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

      ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getStyle().getItemSpacingX());
      if (fallbackWidget.render(!getChildren().isEmpty() && !getTreeWidgetExpanded()))
      {
         setSpecificWidgetOnRowClicked();
         setTreeWidgetExpanded(!getTreeWidgetExpanded());
      }
      ImGui.sameLine();
      ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getStyle().getItemSpacingX());
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