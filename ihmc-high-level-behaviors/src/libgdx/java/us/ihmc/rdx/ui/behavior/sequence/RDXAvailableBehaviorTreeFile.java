package us.ihmc.rdx.ui.behavior.sequence;

import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class RDXAvailableBehaviorTreeFile
{
   private final WorkspaceResourceFile treeFile;
   private String name;

   public RDXAvailableBehaviorTreeFile(WorkspaceResourceFile treeFile)
   {
      this.treeFile = treeFile;
      JSONFileTools.load(treeFile.getFilesystemFile(), jsonNode -> name = jsonNode.get("description").asText());
   }

   public String getName()
   {
      return name;
   }

   public WorkspaceResourceFile getTreeFile()
   {
      return treeFile;
   }
}
