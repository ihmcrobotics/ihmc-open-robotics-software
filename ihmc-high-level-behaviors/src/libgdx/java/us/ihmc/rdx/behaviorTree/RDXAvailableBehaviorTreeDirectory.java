package us.ihmc.rdx.behaviorTree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.behaviorTree.scene.RDXBehaviorTreeScene;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;

public class RDXAvailableBehaviorTreeDirectory
{
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final RDXBehaviorTree behaviorTree;
   private final BehaviorTreeTopologyOperationQueue<RDXBehaviorTreeNode<?, ?>> topologyOperationQueue;
   private final RDXBehaviorTreeScene scene;
   private final Consumer<BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>>> complete;

   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private boolean treeWidgetExpanded = false;
   private final List<RDXAvailableBehaviorTreeFile> indexedTreeFiles = new ArrayList<>();
   private final List<RDXAvailableBehaviorTreeDirectory> indexedTreeDirectories = new ArrayList<>();

   public RDXAvailableBehaviorTreeDirectory(WorkspaceResourceDirectory treeFilesDirectory,
                                            RDXBehaviorTree behaviorTree,
                                            BehaviorTreeTopologyOperationQueue<RDXBehaviorTreeNode<?, ?>> topologyOperationQueue,
                                            RDXBehaviorTreeScene scene,
                                            Consumer<BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>>> complete)
   {
      this.treeFilesDirectory = treeFilesDirectory;
      this.behaviorTree = behaviorTree;
      this.topologyOperationQueue = topologyOperationQueue;
      this.scene = scene;
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
            if (queryContainedFile.getFilesystemFile() == null)
            { // This can happen for partially build workspaces. I'd prefer to avoid the crash which doesn't give much info.
               LogTools.error("Failed to load {}. Filesystem file is null", queryContainedFile.getFileName());
            }
            else
            {
               RDXAvailableBehaviorTreeFile treeFile = new RDXAvailableBehaviorTreeFile(queryContainedFile, scene);
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
      for (WorkspaceResourceDirectory subdirectory : treeFilesDirectory.queryContainedDirectories())
      {
         RDXAvailableBehaviorTreeDirectory subtreeDirectory = new RDXAvailableBehaviorTreeDirectory(subdirectory,
                                                                                                    behaviorTree,
                                                                                                    topologyOperationQueue,
                                                                                                    scene,
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
                  RDXBehaviorTreeRootNode rootNode;
                  if (relativeNode == null) // Automatically add a root node if there isn't one
                     rootNode = (RDXBehaviorTreeRootNode) behaviorTree.getNodeBuilder().createRootNode(behaviorTree.getAndIncrementNextID());
                  else
                     rootNode = behaviorTree.getRootNode();

                  RDXBehaviorTreeNode<?, ?> loadedNode
                        = behaviorTree.getFileLoader().loadFromFile(rootNode, indexedTreeFile.getTreeFile(), topologyOperationQueue);

                  if (loadedNode != null)
                  {
                     RDXBehaviorTreeNode<?, ?> nodeToInsert;
                     if (relativeNode == null) // Add as child of root node if we made one
                     {
                        nodeToInsert = rootNode;
                        rootNode.getDefinition().modify();
                        topologyOperationQueue.queueAppendChildModify(rootNode, loadedNode);
                     }
                     else
                     {
                        nodeToInsert = loadedNode;
                     }

                     BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition
                           = new BehaviorTreeNodeInsertionDefinition<>(insertionType, nodeToInsert, relativeNode);
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
