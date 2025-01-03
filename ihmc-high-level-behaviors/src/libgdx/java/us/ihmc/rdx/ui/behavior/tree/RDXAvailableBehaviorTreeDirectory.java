package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;

public class RDXAvailableBehaviorTreeDirectory
{
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final RDXBehaviorTree tree;
   private final BehaviorTreeTopologyOperationQueue topologyOperationQueue;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final Consumer<BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>>> complete;

   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private boolean treeWidgetExpanded = false;
   private final List<RDXAvailableBehaviorTreeFile> indexedTreeFiles = new ArrayList<>();
   private final List<RDXAvailableBehaviorTreeDirectory> indexedTreeDirectories = new ArrayList<>();

   public RDXAvailableBehaviorTreeDirectory(WorkspaceResourceDirectory treeFilesDirectory,
                                            RDXBehaviorTree tree,
                                            BehaviorTreeTopologyOperationQueue topologyOperationQueue,
                                            ReferenceFrameLibrary referenceFrameLibrary,
                                            Consumer<BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>>> complete)
   {
      this.treeFilesDirectory = treeFilesDirectory;
      this.tree = tree;
      this.topologyOperationQueue = topologyOperationQueue;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.complete = complete;
   }

   public void reindexDirectory()
   {
      indexedTreeFiles.clear();
      indexedTreeDirectories.clear();
      for (WorkspaceResourceFile queryContainedFile : treeFilesDirectory.queryContainedFiles())
      {
         if (queryContainedFile.getFileName().endsWith(".json"))
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
      for (WorkspaceResourceDirectory subdirectory : treeFilesDirectory.queryContainedDirectories())
      {
         RDXAvailableBehaviorTreeDirectory subtreeDirectory = new RDXAvailableBehaviorTreeDirectory(subdirectory,
                                                                                                    tree,
                                                                                                    topologyOperationQueue,
                                                                                                    referenceFrameLibrary,
                                                                                                    complete);
         subtreeDirectory.reindexDirectory();
         indexedTreeDirectories.add(subtreeDirectory);
      }
   }

   public void renderImGuiWidgets(RDXBehaviorTreeNode<?, ?> relativeNode, BehaviorTreeNodeInsertionType insertionType, boolean isRoot)
   {
      if (indexedTreeDirectories.isEmpty() && indexedTreeFiles.isEmpty())
         return;

      for (RDXAvailableBehaviorTreeFile indexedTreeFile : indexedTreeFiles)
      {
         indexedTreeFile.update();
      }
      indexedTreeFiles.sort(Comparator.comparing(treeFile -> treeFile.getTreeFile().getFilesystemFile()));

      if (isRoot)
      {
         treeWidgetExpanded = true;
      }
      else
      {
         ImGui.setCursorPosY(ImGui.getCursorPosY() - ImGui.getFrameHeight() / 6.0f);
         if (expandCollapseRenderer.render(treeWidgetExpanded, false, ImGui.getFrameHeight()))
         {
            treeWidgetExpanded = !treeWidgetExpanded;
         }
         ImGui.sameLine();
         ImGui.setCursorPosY(ImGui.getCursorPosY() + ImGui.getFrameHeight() / 6.0f);
         ImGui.text(treeFilesDirectory.getFilesystemDirectory().getFileName().toString());
      }

      if (treeWidgetExpanded)
      {
         for (RDXAvailableBehaviorTreeDirectory indexedTreeDirectory : indexedTreeDirectories)
         {
            ImGui.indent();
            indexedTreeDirectory.renderImGuiWidgets(relativeNode, insertionType, false);
            ImGui.unindent();
         }

         ImGui.indent();
         for (RDXAvailableBehaviorTreeFile indexedTreeFile : indexedTreeFiles)
         {
            if (ImGuiTools.textWithUnderlineOnHover(indexedTreeFile.getTreeFile().getFileName()))
            {
               if (ImGui.isMouseClicked(ImGuiMouseButton.Left))
               {
                  RDXBehaviorTreeNode<?, ?> loadedNode = tree.getFileLoader().loadFromFile(indexedTreeFile.getTreeFile(), topologyOperationQueue);

                  if (loadedNode != null)
                  {
                     RDXBehaviorTreeNode<?, ?> nodeToInsert = loadedNode;

                     if (tree.getRootNode() == null) // Automatically add a root node if there isn't one
                     {
                        nodeToInsert = new RDXBehaviorTreeRootNode(tree.getBehaviorTreeState().getAndIncrementNextID(),
                                                                   tree.getBehaviorTreeState().getCRDTInfo(),
                                                                   tree.getBehaviorTreeState().getSaveFileDirectory());
                        topologyOperationQueue.queueAddAndFreezeNode(loadedNode, nodeToInsert);
                     }

                     var insertionDefinition = BehaviorTreeNodeInsertionDefinition.build(nodeToInsert,
                                                                                         tree.getBehaviorTreeState(),
                                                                                         tree::setRootNode,
                                                                                         relativeNode,
                                                                                         insertionType);
                     complete.accept(insertionDefinition);
                  }
               }
            }

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
      }
   }
}
