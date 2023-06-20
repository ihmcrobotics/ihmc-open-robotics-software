package us.ihmc.rdx.ui.behavior.editor;

import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class RDXAvailableActionSequence
{
   private final WorkspaceResourceFile sequenceFile;
   private String name;

   public RDXAvailableActionSequence(WorkspaceResourceFile sequenceFile)
   {
      this.sequenceFile = sequenceFile;
      JSONFileTools.load(sequenceFile.getFilesystemFile(), jsonNode -> name = jsonNode.get("name").asText());
   }

   public String getName()
   {
      return name;
   }

   public WorkspaceResourceFile getSequenceFile()
   {
      return sequenceFile;
   }
}
