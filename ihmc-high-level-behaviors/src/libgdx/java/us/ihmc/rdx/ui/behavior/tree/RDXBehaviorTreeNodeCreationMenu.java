package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.behaviors.sequence.actions.PelvisHeightOrientationActionDefinition;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionSequence;
import us.ihmc.rdx.ui.behavior.sequence.RDXAvailableBehaviorTreeFile;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.Comparator;

public class RDXBehaviorTreeNodeCreationMenu
{
   private final RDXBehaviorTree tree;
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final BehaviorTreeTopologyOperationQueue topologyOperationQueue;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<RDXAvailableBehaviorTreeFile> indexedTreeFiles = new ArrayList<>();

   public RDXBehaviorTreeNodeCreationMenu(RDXBehaviorTree tree, WorkspaceResourceDirectory treeFilesDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.tree = tree;
      this.treeFilesDirectory = treeFilesDirectory;
      this.referenceFrameLibrary = referenceFrameLibrary;

      topologyOperationQueue = tree.getBehaviorTreeState().getTopologyChangeQueue();

      reindexDirectory();
   }

   /**
    * This method assumes that the insertion is valid for the relative node.
    * For example, if the insertion requires modifying a parent, we assume it is not null.
    */
   public void renderImGuiWidgets(RDXBehaviorTreeNode<?, ?> relativeNode, BehaviorTreeNodeInsertionType insertionType)
   {
      boolean parentIsActionSequenceNode = relativeNode instanceof RDXActionSequence && insertionType == BehaviorTreeNodeInsertionType.INSERT_AS_CHILD;
      boolean parentIsBasicNode = relativeNode != null
                               && relativeNode.getClass().equals(RDXBehaviorTreeNode.class)
                               && insertionType == BehaviorTreeNodeInsertionType.INSERT_AS_CHILD
                               && RDXBehaviorTreeTools.findActionSequenceAncestor(relativeNode) != null;
      boolean ancestorIsActionSequenceNode = relativeNode instanceof RDXActionNode<?, ?>
                                          && RDXBehaviorTreeTools.findActionSequenceAncestor(relativeNode) != null
            && (insertionType == BehaviorTreeNodeInsertionType.INSERT_AFTER || insertionType == BehaviorTreeNodeInsertionType.INSERT_BEFORE);

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("From file:");
      ImGui.popFont();

      for (RDXAvailableBehaviorTreeFile indexedTreeFile : indexedTreeFiles)
      {
         indexedTreeFile.update();
      }

      indexedTreeFiles.sort(Comparator.comparing((RDXAvailableBehaviorTreeFile file) -> file.getNumberOfFramesInWorld() > 0).reversed()
                                      .thenComparing(RDXAvailableBehaviorTreeFile::getName));

      ImGui.indent();
      for (RDXAvailableBehaviorTreeFile indexedTreeFile : indexedTreeFiles)
      {
         if (indexedTreeFile.getReferenceFramesInWorld().isEmpty())
            ImGui.pushStyleColor(ImGuiCol.Text, ImGui.getColorU32(ImGuiCol.TextDisabled));

         String textToDisplay = "%s".formatted(indexedTreeFile.getTreeFile().getFileName(),
                                                                       indexedTreeFile.getNumberOfFramesInWorld(),
                                                                       indexedTreeFile.getReferenceFrameNames().size());
         if (ImGuiTools.textWithUnderlineOnHover(textToDisplay))
         {
            if (ImGui.isMouseClicked(ImGuiMouseButton.Left))
            {
               try
               {
                  RDXBehaviorTreeNode<?, ?> loadedNode = tree.getFileLoader().loadFromFile(indexedTreeFile, topologyOperationQueue);
                  BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition
                        = BehaviorTreeNodeInsertionDefinition.build(loadedNode, tree.getBehaviorTreeState(), tree::setRootNode, relativeNode, insertionType);

                  complete(insertionDefinition);
               }
               catch (Exception e)
               {
                  LogTools.error("""
                                 Error loading {}.
                                 Please run the JSON sanitizer in debug mode with the NullPointerException breakpoint enabled.
                                 Error: {}
                                 """, textToDisplay, e.getMessage());
               }
            }
         }

         if (indexedTreeFile.getReferenceFramesInWorld().isEmpty())
            ImGui.popStyleColor();

         if (ImGui.isItemHovered())
         {
            ImGui.beginTooltip();

            if (!indexedTreeFile.getNotes().isEmpty())
            {
               ImGui.text(indexedTreeFile.getNotes());
               ImGui.spacing();
            }

            ImGui.text("Reference frames:");

            if (indexedTreeFile.getReferenceFrameNames().isEmpty())
            {
               ImGui.pushStyleColor(ImGuiCol.Text, ImGui.getColorU32(ImGuiCol.TextDisabled));
               ImGui.text("\t(Contains no reference frames.)");
               ImGui.popStyleColor();
            }

            for (String referenceFrameName : indexedTreeFile.getReferenceFrameNames())
            {
               if (!indexedTreeFile.getReferenceFramesInWorld().contains(referenceFrameName))
                  ImGui.pushStyleColor(ImGuiCol.Text, ImGui.getColorU32(ImGuiCol.TextDisabled));

               ImGui.text("\t" + referenceFrameName);

               if (!indexedTreeFile.getReferenceFramesInWorld().contains(referenceFrameName))
                  ImGui.popStyleColor();
            }

            ImGui.endTooltip();
         }
      }
      ImGui.unindent();
      ImGui.spacing();

      ImGui.separator();

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("Control nodes:");
      ImGui.popFont();
      ImGui.indent();

      if (relativeNode != null)
      {
         renderNodeCreationClickable(relativeNode, insertionType, "Basic Node", BehaviorTreeNodeDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Door Traversal", DoorTraversalDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Trash Can Interaction", TrashCanInteractionDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Building Exploration", BuildingExplorationDefinition.class, null);
      }
      if (insertionType == BehaviorTreeNodeInsertionType.INSERT_ROOT)
      {
         renderNodeCreationClickable(relativeNode, insertionType, "Action Sequence", ActionSequenceDefinition.class, null);
      }

      ImGui.unindent();
      ImGui.separator();

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("Actions:");
      ImGui.popFont();
      ImGui.indent();

      renderNodeCreationClickable(relativeNode, insertionType, "Footstep Plan", FootstepPlanActionDefinition.class, null);
      ImGui.text("Foot Pose: ");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), FootPoseActionDefinition.class, side);
      }
      ImGui.text("Hand Pose: ");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), HandPoseActionDefinition.class, side);
      }
      ImGui.text("Sake Hand Command: ");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), SakeHandCommandActionDefinition.class, side);
      }
      renderNodeCreationClickable(relativeNode, insertionType, "Chest Orientation", ChestOrientationActionDefinition.class, null);
      renderNodeCreationClickable(relativeNode, insertionType, "Pelvis Height", PelvisHeightOrientationActionDefinition.class, null);
      renderNodeCreationClickable(relativeNode, insertionType, "Wait", WaitDurationActionDefinition.class, null);
      ImGui.text("Screw Primitive: ");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), ScrewPrimitiveActionDefinition.class, side);
      }
      ImGui.textDisabled("Hand Wrench: ");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), HandWrenchActionDefinition.class, side);
      }

      ImGui.unindent();
   }

   private void renderNodeCreationClickable(RDXBehaviorTreeNode<?, ?> relativeNode,
                                            BehaviorTreeNodeInsertionType insertionType,
                                            String nodeTypeName,
                                            Class<?> nodeType,
                                            @Nullable RobotSide side)
   {
      if (ImGuiTools.textWithUnderlineOnHover(nodeTypeName))
      {
         if (ImGui.isMouseClicked(ImGuiMouseButton.Left))
         {
            RDXBehaviorTreeNode<?, ?> newNode = tree.getNodeBuilder()
                                                    .createNode(nodeType,
                                                                tree.getBehaviorTreeState().getAndIncrementNextID(),
                                                                tree.getBehaviorTreeState().getCRDTInfo(),
                                                                tree.getBehaviorTreeState().getSaveFileDirectory());


            BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition
                  = BehaviorTreeNodeInsertionDefinition.build(newNode, tree.getBehaviorTreeState(), tree::setRootNode, relativeNode, insertionType);

            if (insertionDefinition.getNodeToInsert() instanceof RDXActionNode<?, ?> newAction)
            {
               // We want to to best effort initialization
               RDXActionSequence actionSequenceOrNull = null;
               if (insertionDefinition.getParent() instanceof RDXActionSequence actionSequence)
                  actionSequenceOrNull = actionSequence;
               tree.getNodeBuilder().initializeActionNode(actionSequenceOrNull, newAction, insertionDefinition.getInsertionIndex(), side);
            }

            complete(insertionDefinition);
         }
      }
   }

   private void complete(BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition)
   {
      topologyOperationQueue.queueInsertNode(insertionDefinition);
      ImGui.closeCurrentPopup();

      insertionDefinition.getNodeToInsert().setTreeWidgetExpanded(true);
   }

   public void reindexDirectory()
   {
      indexedTreeFiles.clear();
      for (WorkspaceResourceFile queryContainedFile : treeFilesDirectory.queryContainedFiles())
      {
         RDXAvailableBehaviorTreeFile treeFile = new RDXAvailableBehaviorTreeFile(queryContainedFile, referenceFrameLibrary);
         if (treeFile.getName() != null && treeFile.getNotes() != null)
         {
            indexedTreeFiles.add(treeFile);
         }
         else
         {
            LogTools.error("Failed to load {}", queryContainedFile.getFileName());
         }
      }
   }
}
