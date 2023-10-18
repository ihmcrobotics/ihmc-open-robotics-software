package us.ihmc.behaviors.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

/**
 * A behavior tree node definition that cooresponds to a JSON file.
 */
public class FileBasedBehaviorTreeNodeDefinition extends BehaviorTreeNodeDefinition
{
   private String fileName;

   public FileBasedBehaviorTreeNodeDefinition(WorkspaceResourceFile jsonFile)
   {
      JSONFileTools.load(jsonFile, this::loadFromFile);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);
   }

   public void setFileName(String fileName)
   {
      this.fileName = fileName;
   }
}
