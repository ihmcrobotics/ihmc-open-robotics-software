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

/**
 * @param <T> The generic type of this node: RDX or Executor
 */
public class BehaviorTreeFileLoader<T extends BehaviorTreeNode<T, ? ,?>>
{
   private final BehaviorTree<?, T> behaviorTree;
   private final BehaviorTreeNodeBuilder<T> nodeBuilder;
   private final WorkspaceResourceDirectory treeFilesDirectory;

   public BehaviorTreeFileLoader(BehaviorTree<?, T> behaviorTree,
                                 BehaviorTreeNodeBuilder<T> nodeBuilder,
                                 WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.behaviorTree = behaviorTree;
      this.nodeBuilder = nodeBuilder;
      this.treeFilesDirectory = treeFilesDirectory;
   }

   public T loadFromFile(BehaviorTreeRootNode<T> rootNode, WorkspaceResourceFile file, BehaviorTreeTopologyOperationQueue<T> topologyOperationQueue)
   {
      return loadFromFile(rootNode, file, null, null, topologyOperationQueue);
   }

   private T loadFromFile(BehaviorTreeRootNode<T> rootNode,
                          WorkspaceResourceFile file,
                          JsonNode jsonNode,
                          T parentNode,
                          BehaviorTreeTopologyOperationQueue<T> topologyOperationQueue)
   {
      MutableObject<T> loadedNode = new MutableObject<>();

      if (jsonNode == null)
      {
         try
         {
            // Try loading from file first, since maybe the user saved a new version
            Path filesystemFile = file.getFilesystemFile();
            if (filesystemFile != null && Files.exists(filesystemFile))
            {
               LogTools.info("Loading from file: {}", filesystemFile);
               JSONFileTools.load(file, childJsonNode ->
                     loadedNode.setValue(loadFromFile(rootNode, file, childJsonNode, parentNode, topologyOperationQueue)));
            }
            else
            {
               URL classpathResource = file.getClasspathResource();
               if (classpathResource != null)
               {
                  LogTools.info("Loading from resource: {}", classpathResource);
                  JSONFileTools.load(classpathResource, childJsonNode ->
                        loadedNode.setValue(loadFromFile(rootNode, file, childJsonNode, parentNode, topologyOperationQueue)));
               }
            }
         }
         catch (Exception e)
         {
            LogTools.error("""
                           Error loading {}.
                           Please run the JSON sanitizer in debug mode with the NullPointerException breakpoint enabled.
                           Error: {}
                           """, file.getFileName(), e.getMessage());
         }
      }
      else
      {
         String typeName = jsonNode.get("type").textValue();

         T node = nodeBuilder.createNode(BehaviorTreeDefinitionRegistry.getClassFromTypeName(typeName),
                                         behaviorTree.getAndIncrementNextID(),
                                         rootNode);
         node.getDefinition().modify();
         node.getDefinition().loadFromFile(jsonNode);

         // Make sure the node is named the same as the file including subdirectory
         if (node.getDefinition().isJSONRoot())
         {
            String relativePathString;
            Path filesystemFile = file.getFilesystemFile();
            if (filesystemFile != null && Files.exists(filesystemFile))
            {
               relativePathString = treeFilesDirectory.getFilesystemDirectory().relativize(filesystemFile).toString();
            }
            else
            {
               String classpathResourceString = file.getClasspathResource().getPath();
               String pathNecessaryForClasspathLoading = treeFilesDirectory.getPathNecessaryForClasspathLoading();

               relativePathString = classpathResourceString.substring(classpathResourceString.lastIndexOf(pathNecessaryForClasspathLoading)
                                                                      + pathNecessaryForClasspathLoading.length() + 1); // Include ending '/'
            }
            if (!node.getDefinition().getName().equals(relativePathString))
            {
               LogTools.warn("Renaming node to match file name: {} -> {}", node.getDefinition().getName(), relativePathString);
               node.getDefinition().setName(relativePathString);
            }
         }

         LogTools.info("Creating node: {}:{}", node.getDefinition().getName(), node.getState().getID());

         if (parentNode != null)
         {
            topologyOperationQueue.queueAppendChildModify(parentNode, node);
         }

         JSONTools.forEachArrayElement(jsonNode, "children", childJsonNode ->
         {
            if (childJsonNode.has("file"))
            {
               WorkspaceResourceFile childFile = new WorkspaceResourceFile(treeFilesDirectory, childJsonNode.get("file").asText());
               loadFromFile(rootNode, childFile, null, node, topologyOperationQueue);
            }
            else
            {
               loadFromFile(rootNode, file, childJsonNode, node, topologyOperationQueue);
            }
         });

         // Set the on disk fields after the children are added so we
         // can mark modified when children topology changes
         topologyOperationQueue.queueOperation(() -> node.getDefinition().setOnDiskFields());

         loadedNode.setValue(node);
      }

      return loadedNode.getValue();
   }
}
