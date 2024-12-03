package us.ihmc.behaviors.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;

public class BehaviorTreeFileLoader<T extends BehaviorTreeNodeLayer<T, ?, ?, ?>>
{
   private final BehaviorTreeState behaviorTreeState;
   private final BehaviorTreeNodeStateBuilder<T> nodeBuilder;
   private final WorkspaceResourceDirectory treeFilesDirectory;

   public BehaviorTreeFileLoader(BehaviorTreeState behaviorTreeState,
                                 BehaviorTreeNodeStateBuilder<T> nodeBuilder,
                                 WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.nodeBuilder = nodeBuilder;
      this.treeFilesDirectory = treeFilesDirectory;
   }

   public T loadFromFile(WorkspaceResourceFile fileToLoad, BehaviorTreeTopologyOperationQueue topologyOperationQueue)
   {
      MutableObject<T> loadedNode = new MutableObject<>();

      try
      {
         // Try loading from file first, since maybe the user saved a new version
         Path file = fileToLoad.getFilesystemFile();
         if (file != null && Files.exists(file))
         {
            LogTools.info("Loading from file: {}", file);
            String relativePath = treeFilesDirectory.getFilesystemDirectory().relativize(file).toString();
            JSONFileTools.load(fileToLoad, jsonNode -> loadedNode.setValue(loadFromFile(relativePath, jsonNode, null, topologyOperationQueue)));
         }
         else
         {
            URL classpathResource = fileToLoad.getClasspathResource();
            if (classpathResource != null)
            {
               LogTools.info("Loading from resource: {}", classpathResource);
               String relativePath = classpathResource.toString().replaceAll(treeFilesDirectory.getPathNecessaryForClasspathLoading(), "");
               JSONFileTools.load(classpathResource, jsonNode -> loadedNode.setValue(loadFromFile(relativePath, jsonNode, null, topologyOperationQueue)));
            }
         }
      }
      catch (Exception e)
      {
         LogTools.error("""
                        Error loading {}.
                        Please run the JSON sanitizer in debug mode with the NullPointerException breakpoint enabled.
                        Error: {}
                        """, fileToLoad.getFileName(), e.getMessage());
      }

      return loadedNode.getValue();
   }

   private T loadFromFile(String filePath, JsonNode jsonNode, T parentNode, BehaviorTreeTopologyOperationQueue topologyOperationQueue)
   {
      String typeName = jsonNode.get("type").textValue();

      T node = nodeBuilder.createNode(BehaviorTreeDefinitionRegistry.getClassFromTypeName(typeName),
                                      behaviorTreeState.getAndIncrementNextID(),
                                      behaviorTreeState.getCRDTInfo(),
                                      behaviorTreeState.getSaveFileDirectory());
      node.getDefinition().loadFromFile(jsonNode);

      if (node.getDefinition().isJSONRoot() && !node.getDefinition().getName().equals(filePath))
      {
         LogTools.warn("Renaming node to match file name: {} -> {}", node.getDefinition().getName(), filePath);
         node.getDefinition().setName(filePath);
      }

      LogTools.info("Creating node: {}:{}", node.getDefinition().getName(), node.getState().getID());

      if (parentNode != null)
      {
         topologyOperationQueue.queueAddAndFreezeNode(node, parentNode);
      }

      JSONTools.forEachArrayElement(jsonNode, "children", childJsonNode ->
      {
         JsonNode fileNode = childJsonNode.get("file");
         if (fileNode == null)
         {
            loadFromFile(filePath, childJsonNode, node, topologyOperationQueue);
         }
         else
         {
            WorkspaceResourceFile childFile = new WorkspaceResourceFile(behaviorTreeState.getSaveFileDirectory(), fileNode.asText());
            LogTools.info("Loading {}", childFile.getFilesystemFile());
            String relativePath = treeFilesDirectory.getFilesystemDirectory().relativize(childFile.getFilesystemFile()).toString();
            JSONFileTools.load(childFile, childJSONNode ->
            {
               loadFromFile(relativePath, childJSONNode, node, topologyOperationQueue);
            });
         }
      });

      // Set the on disk fields after the children are added so we
      // can mark modified when children topology changes
      topologyOperationQueue.queueOperation(() -> node.getDefinition().setOnDiskFields());

      return node;
   }
}
