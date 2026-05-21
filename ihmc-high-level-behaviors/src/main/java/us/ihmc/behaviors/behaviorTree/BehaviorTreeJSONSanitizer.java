package us.ihmc.behaviors.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.log.LogTools;
import us.ihmc.jros2.ROS2Node;
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
   private final CRDTInfo crdtInfo;
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final DRCRobotModel robotModel;

   public BehaviorTreeJSONSanitizer(Class<?> classForFindingSourceSetDirectory, DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;

      ROS2Node ros2Node = new ROS2Node("json_sanitizer");
      ROS2PeerClockOffsetEstimator peerClockEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, peerClockEstimator);

      treeFilesDirectory = new WorkspaceResourceDirectory(classForFindingSourceSetDirectory, "/behaviorTrees");

      processDirectory(treeFilesDirectory);

      peerClockEstimator.destroy();
      ros2Node.close();
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
            BehaviorTreeNodeDefinition finalParent = parentNode;
            if (filesystemFile != null && Files.exists(filesystemFile))
            {
               LogTools.info("Loading from file: {}", filesystemFile);
               JSONFileTools.load(file, childJsonNode ->
                     loadedNode.setValue(loadFromFile(file, childJsonNode, finalParent)));
            }
            else
            {
               URL classpathResource = file.getClasspathResource();
               if (classpathResource != null)
               {
                  LogTools.info("Loading from resource: {}", classpathResource);
                  JSONFileTools.load(classpathResource, childJsonNode ->
                        loadedNode.setValue(loadFromFile(file, childJsonNode, finalParent)));
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

         if (parentNode == null)
            parentNode = BehaviorTreeDefinitionBuilder.createRootNode(crdtInfo, treeFilesDirectory, robotModel);

         BehaviorTreeRootNodeDefinition rootNode = BehaviorTreeTools.findRootNode(parentNode);
         BehaviorTreeNodeDefinition node = BehaviorTreeDefinitionBuilder.createNode(definitionType, rootNode);

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

         node.setParent(parentNode);
         parentNode.getChildren().add(parentNode.getChildren().size(), node);

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
