package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionSequence;

public class RDXBehaviorTreeWidgetsVerticalLayout
{
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private final RDXBehaviorTree tree;
   private final BehaviorTreeTopologyOperationQueue topologyOperationQueue;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private BehaviorTreeNodeInsertionType insertionType = null;
   private RDXBehaviorTreeNode<?, ?> modalPopupNode;
   private final TypedNotification<Runnable> queuePopupModal = new TypedNotification<>();

   public RDXBehaviorTreeWidgetsVerticalLayout(RDXBehaviorTree tree)
   {
      this.tree = tree;

      topologyOperationQueue = tree.getBehaviorTreeState().getTopologyChangeQueue();
   }

   public void renderImGuiWidgets(RDXBehaviorTreeNode<?, ?> node)
   {
      ImGui.selectable("Selectable");
      ImGui.sameLine();

      if (expandCollapseRenderer.render(node.getTreeWidgetExpanded()))
      {
         node.setTreeWidgetExpanded(!node.getTreeWidgetExpanded());
      }

      ImGui.sameLine();
      node.renderTreeViewIconArea();

      ImGui.sameLine();
      node.renderNodeDescription();

      if (ImGui.beginPopup(node.getNodePopupID()))
      {
         node.renderContextMenuItems();

         ImGui.separator();
         if (!node.isRootNode() && !(node instanceof RDXActionSequence))
         {
            if (ImGui.menuItem(labels.get("Insert Node Before...")))
            {
               queuePopupModal.set(() -> popNodeCreationModalDialog(node, BehaviorTreeNodeInsertionType.INSERT_BEFORE));
            }
            if (ImGui.menuItem(labels.get("Insert Node After...")))
            {
               queuePopupModal.set(() -> popNodeCreationModalDialog(node, BehaviorTreeNodeInsertionType.INSERT_AFTER));
            }
         }
         if (node.getChildren().isEmpty() && !(node instanceof RDXActionNode<?, ?>))
         {
            if (ImGui.menuItem(labels.get("Add Child Node...")))
            {
               queuePopupModal.set(() -> popNodeCreationModalDialog(node, BehaviorTreeNodeInsertionType.INSERT_AS_CHILD));
            }
         }
         if (!(node.isRootNode() && !node.getChildren().isEmpty()))
         {
            ImGui.separator();
         }

         if (node.getParent() != null)
         {
            if (ImGui.beginMenu(labels.get("Move to Before")))
            {
               renderMoveRelativeItems(node, BehaviorTreeNodeInsertionType.INSERT_BEFORE);
               ImGui.endMenu();
            }
            if (ImGui.beginMenu(labels.get("Move to After")))
            {
               renderMoveRelativeItems(node, BehaviorTreeNodeInsertionType.INSERT_AFTER);
               ImGui.endMenu();
            }

            ImGui.separator();
         }

         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
         if (ImGui.menuItem(labels.get("Delete Node")))
         {
            topologyOperationQueue.queueDestroySubtree(node);

            if (node.isRootNode()) // Root node
            {
               tree.setRootNode(null);
               tree.getBehaviorTreeState().freeze();
            }
         }
         ImGui.popStyleColor();

         ImGui.separator();
         if (ImGui.menuItem(labels.get("Cancel")))
            ImGui.closeCurrentPopup();

         ImGui.endPopup();
      }

      // This doesn't work if done inside the other popup
      if (queuePopupModal.poll())
         queuePopupModal.read().run();

      renderNodeCreationModalDialog(node);

      if (node.getTreeWidgetExpanded())
      {
         float indentAmount = 10.0f;
         ImGui.indent(indentAmount);

         node.renderImGuiWidgets();

         for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         {
            renderImGuiWidgets(child);
         }

         ImGui.unindent(indentAmount);
      }
   }

   private void popNodeCreationModalDialog(RDXBehaviorTreeNode<?, ?> node, BehaviorTreeNodeInsertionType insertionType)
   {
      this.modalPopupNode = node;
      this.insertionType = insertionType;

      switch (insertionType)
      {
         case INSERT_BEFORE -> node.setModalPopupTitle("Insert before \"%s\"".formatted(node.getDefinition().getDescription()));
         case INSERT_AFTER -> node.setModalPopupTitle("Insert after \"%s\"".formatted(node.getDefinition().getDescription()));
         case INSERT_AS_CHILD -> node.setModalPopupTitle("Insert as child of \"%s\"".formatted(node.getDefinition().getDescription()));
      }

      ImGui.openPopup(node.getModalPopupID());
      LogTools.info("Opening popup {}", node.getModalPopupID());
   }

   private void renderNodeCreationModalDialog(RDXBehaviorTreeNode<?, ?> node)
   {
      if (ImGui.beginPopupModal(node.getModalPopupID()))
      {
         tree.getNodeCreationMenu().renderImGuiWidgets(modalPopupNode, insertionType);

         ImGui.separator();
         if (ImGui.button(labels.get("Cancel")) || ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
         {
            ImGui.closeCurrentPopup();
         }
         ImGui.endPopup();
      }
   }

   private void renderMoveRelativeItems(RDXBehaviorTreeNode<?, ?> nodeToMove, BehaviorTreeNodeInsertionType insertionType)
   {
      RDXBehaviorTreeNode<?, ?> rootNode = RDXBehaviorTreeTools.findRootNode(nodeToMove);
      RDXBehaviorTreeTools.runForSubtreeNodes(rootNode, relativeNode ->
      {
         if (relativeNode != nodeToMove && relativeNode != rootNode)
         {
            if (ImGui.menuItem(relativeNode.getDefinition().getDescription()))
            {
               topologyOperationQueue.queueMoveAndFreezeNode(nodeToMove, nodeToMove.getParent(), relativeNode.getParent(), relativeNode, insertionType);
            }
         }
      });
   }
}
