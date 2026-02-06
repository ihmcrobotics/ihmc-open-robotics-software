package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXLeafNode;
import us.ihmc.rdx.ui.widgets.ImGuiFallbackWidget;

import java.util.ArrayList;
import java.util.List;

public class RDXFallbackNode extends RDXBehaviorTreeNode<FallbackNodeState, FallbackNodeDefinition>
{
   private final ImGuiFallbackWidget fallbackWidget = new ImGuiFallbackWidget();

   private final List<RDXLeafNode<?, ?>> childrenLeaves = new ArrayList<>();
   private final List<RDXLeafNode<?, ?>> tryLeaves = new ArrayList<>();
   private final List<RDXLeafNode<?, ?>> catchLeaves = new ArrayList<>();

   public RDXFallbackNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new FallbackNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      childrenLeaves.clear();
      for (RDXBehaviorTreeNode<?, ?> child : getChildren())
         if (child instanceof RDXLeafNode<?, ?> leafNode)
            childrenLeaves.add(leafNode);

      tryLeaves.clear();
      catchLeaves.clear();
      if (!childrenLeaves.isEmpty())
      {
         int firstLeafIndex = childrenLeaves.get(0).getState().getLeafIndex();

         for (RDXLeafNode<?, ?> child : childrenLeaves)
            if (child.getState().getExecuteAfterLeafIndex() < firstLeafIndex)
               tryLeaves.add(child);
            else
               catchLeaves.add(child);
      }
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();

      fallbackWidget.render();

      ImGui.sameLine();
      super.renderEditableName();
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

   public List<RDXLeafNode<?, ?>> getChildrenLeaves()
   {
      return childrenLeaves;
   }

   public List<RDXLeafNode<?, ?>> getTryLeaves()
   {
      return tryLeaves;
   }

   public List<RDXLeafNode<?, ?>> getCatchLeaves()
   {
      return catchLeaves;
   }
}