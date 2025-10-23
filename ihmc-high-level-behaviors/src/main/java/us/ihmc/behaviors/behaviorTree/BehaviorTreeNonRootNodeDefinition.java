package us.ihmc.behaviors.behaviorTree;

import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

/**
 * The base definition of a non-root behavior tree node.
 * This exists to provide a protected reference to the root node.
 */
public class BehaviorTreeNonRootNodeDefinition extends BehaviorTreeNodeDefinition
{
   protected final BehaviorTreeRootNodeDefinition rootNode;
   private final WorkspaceResourceDirectory saveFileDirectory;

   public BehaviorTreeNonRootNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode.getCRDTInfo());

      this.rootNode = rootNode;

      saveFileDirectory = rootNode.getSaveFileDirectory();
   }

   /** Save as JSON file root node. */
   public void saveToFile()
   {
      if (!isJSONRoot())
         LogTools.error("Cannot save. Can only be called for JSON roots.");

      WorkspaceResourceFile saveFile = new WorkspaceResourceFile(saveFileDirectory, getName());
      LogTools.info("Saving behavior tree: {}", saveFile.getFilesystemFile());
      if (JSONFileTools.save(saveFile, this::saveToFile)) // Success
      {
         BehaviorTreeTools.runForSubtreeNodes(this, BehaviorTreeNodeDefinition::setOnDiskFields);
      }
   }
}
