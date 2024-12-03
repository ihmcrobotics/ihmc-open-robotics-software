package us.ihmc.behaviors.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperations;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Tool to load all JSON files and resave them all in order to perform
 * schema changes.
 */
public class BehaviorTreeJSONSanitizer
{
   private final CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, 1);
   private final WorkspaceResourceDirectory treeFilesDirectory;

   public BehaviorTreeJSONSanitizer(Class<?> classForFindingSourceSetDirectory)
   {
      treeFilesDirectory = new WorkspaceResourceDirectory(classForFindingSourceSetDirectory, "/behaviorTrees");

      processDirectory(treeFilesDirectory);
   }

   private void processDirectory(WorkspaceResourceDirectory directory)
   {
      for (WorkspaceResourceFile file : directory.queryContainedFiles())
      {
         if (file.getFileName().endsWith(".json"))
         {
            BehaviorTreeNodeDefinition loadedNode = loadFromFile(file, null, null);
            loadedNode.saveToFile();
         }
      }
      for (WorkspaceResourceDirectory subdirectory : directory.queryContainedDirectories())
      {
         processDirectory(subdirectory);
      }
   }

   private BehaviorTreeNodeDefinition loadFromFile(WorkspaceResourceFile file, JsonNode jsonNode, BehaviorTreeNodeDefinition parentNode)
   {
      MutableObject<BehaviorTreeNodeDefinition> loadedNode = new MutableObject<>();

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
                     loadedNode.setValue(loadFromFile(file, childJsonNode, parentNode)));
            }
            else
            {
               URL classpathResource = file.getClasspathResource();
               if (classpathResource != null)
               {
                  LogTools.info("Loading from resource: {}", classpathResource);
                  JSONFileTools.load(classpathResource, childJsonNode ->
                        loadedNode.setValue(loadFromFile(file, childJsonNode, parentNode)));
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

         Class<?> definitionType = BehaviorTreeDefinitionRegistry.getClassFromTypeName(typeName);

         BehaviorTreeNodeDefinition node = BehaviorTreeDefinitionBuilder.createNode(definitionType, crdtInfo, treeFilesDirectory);

         node.loadFromFile(jsonNode);

         // Make sure the node is named the same as the file including subdirectory
         if (node.isJSONRoot())
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
            if (!node.getName().equals(relativePathString))
            {
               LogTools.warn("Renaming node to match file name: {} -> {}", node.getName(), relativePathString);
               node.setName(relativePathString);
            }
         }

         LogTools.info("Creating node: {}", node.getName());

         if (parentNode != null)
         {
            BehaviorTreeTopologyOperations.addChildBasic(node, parentNode);
         }

         JSONTools.forEachArrayElement(jsonNode, "children", childJsonNode ->
         {
            if (childJsonNode.has("file"))
            {
               WorkspaceResourceFile childFile = new WorkspaceResourceFile(treeFilesDirectory, childJsonNode.get("file").asText());
               loadFromFile(childFile, null, node);
            }
            else
            {
               loadFromFile(file, childJsonNode, node);
            }
         });

         loadedNode.setValue(node);
      }

      return loadedNode.getValue();
   }
}
