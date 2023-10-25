package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXAvailableBehaviorTreeFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.Comparator;

public class RDXBehaviorTreeNodesMenu
{
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final Notification menuShouldClose = new Notification();
   private final ArrayList<RDXAvailableBehaviorTreeFile> indexedTreeFiles = new ArrayList<>();
   private final TypedNotification<RDXAvailableBehaviorTreeFile> loadFileRequest = new TypedNotification<>();

   public RDXBehaviorTreeNodesMenu(WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.treeFilesDirectory = treeFilesDirectory;

      reindexDirectory();
   }

   private void reindexDirectory()
   {
      indexedTreeFiles.clear();
      for (WorkspaceResourceFile queryContainedFile : treeFilesDirectory.queryContainedFiles())
      {
         indexedTreeFiles.add(new RDXAvailableBehaviorTreeFile(queryContainedFile));
      }

      // Keep them in alphabetical order
      indexedTreeFiles.sort(Comparator.comparing(RDXAvailableBehaviorTreeFile::getName));
   }

   public void renderNodesMenu()
   {
      if (ImGui.beginMenu(labels.get("Nodes"), !menuShouldClose.poll()))
      {
         ImGui.text("From File:");

         for (RDXAvailableBehaviorTreeFile indexedTreeFile : indexedTreeFiles)
         {
            if (ImGui.button(indexedTreeFile.getName()))
            {

            }
         }

         if (ImGui.button(labels.get("Reindex directory")))
         {
            reindexDirectory();
         }

         ImGui.text("Control Nodes:");
         // TODO

         ImGui.text("Actions");
         // TODO

         ImGui.endMenu();
      }
   }

   public TypedNotification<RDXAvailableBehaviorTreeFile> getLoadFileRequest()
   {
      return loadFileRequest;
   }
}
