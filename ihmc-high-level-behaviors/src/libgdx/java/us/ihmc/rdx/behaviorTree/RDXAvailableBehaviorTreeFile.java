package us.ihmc.rdx.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class RDXAvailableBehaviorTreeFile
{
   private final WorkspaceResourceFile treeFile;
   private String name;
   private String notes;

   public RDXAvailableBehaviorTreeFile(WorkspaceResourceFile treeFile)
   {
      this.treeFile = treeFile;

      JSONFileTools.load(treeFile.getFilesystemFile(), this::loadFromFile);
   }

   private void loadFromFile(JsonNode jsonNode)
   {
      name = jsonNode.get("name").asText();
      notes = jsonNode.get("notes").asText();
   }

   public String getName()
   {
      return name;
   }

   public String getNotes()
   {
      return notes;
   }

   public WorkspaceResourceFile getTreeFile()
   {
      return treeFile;
   }
}
