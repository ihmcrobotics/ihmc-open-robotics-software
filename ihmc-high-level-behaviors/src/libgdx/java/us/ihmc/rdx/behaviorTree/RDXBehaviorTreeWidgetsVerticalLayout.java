package us.ihmc.rdx.behaviorTree;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.behaviorTree.actions.RDXActionNode;

public class RDXBehaviorTreeWidgetsVerticalLayout
{
   private final RDXBehaviorTree behaviorTree;
   private final BehaviorTreeTopologyOperationQueue<RDXBehaviorTreeNode<?, ?>> topologyOperationQueue;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private BehaviorTreeNodeInsertionType insertionType = null;
   private RDXBehaviorTreeNode<?, ?> modalPopupNode;
   private final TypedNotification<Runnable> queuePopupModal = new TypedNotification<>();
   private RDXBehaviorTreeNode<?, ?> draggedNode = null;
   private RDXBehaviorTreeNode<?, ?> lastRendereredNode = null;

   public RDXBehaviorTreeWidgetsVerticalLayout(RDXBehaviorTree behaviorTree)
   {
      this.behaviorTree = behaviorTree;

      topologyOperationQueue = behaviorTree.getTopologyChangeQueue();
   }

   public void renderImGuiWidgets()
   {
      lastRendereredNode = null;
      renderImGuiWidgets(behaviorTree.getRootNode());

      if (!ImGui.isMouseDown(ImGuiMouseButton.Left))
         draggedNode = null;
   }

   private void renderImGuiWidgets(RDXBehaviorTreeNode<?, ?> node)
   {
      ImGui.pushStyleVar(ImGuiStyleVar.ItemSpacing, ImGui.getStyle().getItemSpacingX(), 0.0f);

      node.setDraggedNode(draggedNode);
      node.renderTreeViewRow();

      if (draggedNode == null && node.getDragging())
         draggedNode = node;
      if (node.getDragReleasedBefore())
      {
         RDXBaseUI.pushNotification("Moved %s to before %s".formatted(draggedNode.getDefinition().getName(), node.getDefinition().getName()));
         topologyOperationQueue.queueMoveChildModify(draggedNode.getParent(), node.getParent(), draggedNode, node, BehaviorTreeNodeInsertionType.INSERT_BEFORE);
         draggedNode = null;
      }
      else if (node.getDragReleasedAfter())
      {
         RDXBaseUI.pushNotification("Moved %s to after %s".formatted(draggedNode.getDefinition().getName(), node.getDefinition().getName()));
         topologyOperationQueue.queueMoveChildModify(draggedNode.getParent(), node.getParent(), draggedNode, node, BehaviorTreeNodeInsertionType.INSERT_AFTER);
         draggedNode = null;
      }

      ImGui.popStyleVar();

      if (ImGui.beginPopup(node.getNodePopupID()))
      {
         node.renderContextMenuItems();

         ImGui.separator();
         if (!node.isRootNode())
         {
            if (ImGui.menuItem(labels.get("Insert Node Before...")))
            {
               queuePopupModal.set(() -> popNodeCreationModalDialog(node, BehaviorTreeNodeInsertionType.INSERT_BEFORE));
            }
            if (ImGui.menuItem(labels.get("Insert Node After...")))
            {
               queuePopupModal.set(() -> popNodeCreationModalDialog(node, BehaviorTreeNodeInsertionType.INSERT_AFTER));
            }
            if (node instanceof RDXLeafNode<?, ?> && ImGui.menuItem(labels.get("Duplicate Node")))
            {
               BehaviorTreeNodeInsertionType insertionType = BehaviorTreeNodeInsertionType.INSERT_AFTER;
               Class<?> nodeType = node.getDefinition().getClass();
               RDXBehaviorTreeNode<?, ?> newNode = behaviorTree.getNodeBuilder().createNode(nodeType,
                                                                                            behaviorTree.getAndIncrementNextID(),
                                                                                            behaviorTree.getRootNode());
               ObjectMapper mapper = new ObjectMapper();
               ObjectNode saveNode = mapper.createObjectNode();
               node.getDefinition().saveToFile(saveNode); // exploit the JSON methods to copy the definition
               ObjectNode loadNode = (ObjectNode) ExceptionTools.handle(() ->
                            mapper.readTree(mapper.writeValueAsString(saveNode)), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
               newNode.getDefinition().loadFromFile(loadNode);
               newNode.getDefinition().setName(node.getDefinition().getName() + " (Copy)");
               topologyOperationQueue.queueInsertNodeModify(new BehaviorTreeNodeInsertionDefinition<>(insertionType, newNode, node));
            }
         }
         if (!(node instanceof RDXActionNode<?, ?>))
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
            if (ImGui.beginMenu(labels.get("Move to Child of")))
            {
               renderMoveRelativeItems(node, BehaviorTreeNodeInsertionType.INSERT_AS_CHILD);
               ImGui.endMenu();
            }

            ImGui.separator();
         }

         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
         if (ImGui.menuItem(labels.get("Delete Node")))
         {
            if (node.isRootNode())
               topologyOperationQueue.queueDestroyEntireTreeModify();
            else
            {
               if (node.getSelected() && lastRendereredNode != null) // Select previous node so layout doesn't jump
                  lastRendereredNode.setSelected();

               int executionNextIndex = behaviorTree.getRootNode().getState().getExecutionNextIndex();
               if (node.getState().getDepthFirstIndex() < executionNextIndex) // Keep the execution next index set to the same node
                  behaviorTree.getRootNode().getState().setExecutionNextIndex(executionNextIndex - 1);

               topologyOperationQueue.queueDestroySubtreeModify(node);
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

      lastRendereredNode = node;
      if (node.getTreeWidgetExpanded())
      {
         float indentAmount = ImGui.getFontSize() * 0.7f;
         ImGui.indent(indentAmount);

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
         case INSERT_BEFORE -> node.setModalPopupTitle("Insert before \"%s\"".formatted(node.getDefinition().getName()));
         case INSERT_AFTER -> node.setModalPopupTitle("Insert after \"%s\"".formatted(node.getDefinition().getName()));
         case INSERT_AS_CHILD -> node.setModalPopupTitle("Insert as child of \"%s\"".formatted(node.getDefinition().getName()));
      }

      // Update listings every time we pop the node creation dialog
      behaviorTree.getNodeCreationMenu().reindexDirectory();

      ImGui.openPopup(node.getModalPopupID());
      LogTools.info("Opening popup {}", node.getModalPopupID());
   }

   private void renderNodeCreationModalDialog(RDXBehaviorTreeNode<?, ?> node)
   {
      // Make sure the menu doesn't grow to be taller than the main window
      // and keep it in the main viewport.
      // We can get a native crash if this popup creates its own viewport.
      ImGui.setNextWindowViewport(ImGui.getMainViewport().getID());
      int windowFlags = ImGuiWindowFlags.None;
      if (ImGui.beginPopupModal(node.getModalPopupID(), windowFlags))
      {
         behaviorTree.getNodeCreationMenu().renderImGuiWidgets(modalPopupNode, insertionType);

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
            if (insertionType == BehaviorTreeNodeInsertionType.INSERT_AS_CHILD)
            {
               if (!(relativeNode instanceof RDXActionNode))
               {
                  if (ImGui.menuItem(relativeNode.getDefinition().getName()))
                  {
                     topologyOperationQueue.queueMoveChildModify(nodeToMove.getParent(), relativeNode, nodeToMove, relativeNode, insertionType);
                  }
               }
            }
            else
            {
               if (ImGui.menuItem(relativeNode.getDefinition().getName()))
               {
                  topologyOperationQueue.queueMoveChildModify(nodeToMove.getParent(), relativeNode.getParent(), nodeToMove, relativeNode, insertionType);
               }
            }
         }
      });
   }
}
