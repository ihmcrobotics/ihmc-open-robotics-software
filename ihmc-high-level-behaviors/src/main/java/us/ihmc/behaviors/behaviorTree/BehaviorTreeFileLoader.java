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

   public T loadFromFile(WorkspaceResourceFile file, BehaviorTreeTopologyOperationQueue topologyOperationQueue)
   {
      return loadFromFile(file, null, null, topologyOperationQueue);
   }

   private T loadFromFile(WorkspaceResourceFile file, JsonNode jsonNode, T parentNode, BehaviorTreeTopologyOperationQueue topologyOperationQueue)
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
                     loadedNode.setValue(loadFromFile(file, childJsonNode, parentNode, topologyOperationQueue)));
            }
            else
            {
               URL classpathResource = file.getClasspathResource();
               if (classpathResource != null)
               {
                  LogTools.info("Loading from resource: {}", classpathResource);
                  JSONFileTools.load(classpathResource, childJsonNode ->
                        loadedNode.setValue(loadFromFile(file, childJsonNode, parentNode, topologyOperationQueue)));
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
                                         behaviorTreeState.getAndIncrementNextID(),
                                         behaviorTreeState.getCRDTInfo(),
                                         behaviorTreeState.getSaveFileDirectory());
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
            topologyOperationQueue.queueAddAndFreezeNode(node, parentNode);
         }

         JSONTools.forEachArrayElement(jsonNode, "children", childJsonNode ->
         {
            if (childJsonNode.has("file"))
            {
               WorkspaceResourceFile childFile = new WorkspaceResourceFile(behaviorTreeState.getSaveFileDirectory(), childJsonNode.get("file").asText());
               loadFromFile(childFile, null, node, topologyOperationQueue);
            }
            else
            {
               loadFromFile(file, childJsonNode, node, topologyOperationQueue);
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
