package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalString;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * The base definition of a behavior tree node is just a
 * human readable description and a list of children.
 */
public class BehaviorTreeNodeDefinition implements BehaviorTreeNode<BehaviorTreeNodeDefinition>
{
   /** A human readable description of what the node does */
   private final CRDTUnidirectionalString description;
   /** Behavior tree children node definitions. */
   private final List<BehaviorTreeNodeDefinition> children = new ArrayList<>();
   private transient BehaviorTreeNodeDefinition parent;
   private final WorkspaceResourceDirectory saveFileDirectory;
   /** Empty string if not the root of a JSON file */
   private final CRDTUnidirectionalString jsonFileName;

   public BehaviorTreeNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      this.saveFileDirectory = saveFileDirectory;

      description = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, "");
      jsonFileName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, "");
   }

   /** Save as JSON file root node. */
   public void saveToFile()
   {
      if (!isJSONRoot())
         LogTools.error("Cannot save. Can only be called for JSON roots.");

      WorkspaceResourceFile saveFile = new WorkspaceResourceFile(saveFileDirectory, jsonFileName.getValue());
      LogTools.info("Saving behavior tree: {}", saveFile.getFilesystemFile());
      JSONFileTools.save(saveFile, this::saveToFile);
   }

   /**
    * Saves the file recursively.
    */
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("type", getClass().getSimpleName());

      if (!description.getValue().isEmpty()) // No reason to write default description
         jsonNode.put("description", description.getValue());

      ArrayNode childrenArrayJsonNode = jsonNode.putArray("children");
      for (BehaviorTreeNodeDefinition child : children)
      {
         ObjectNode childJsonNode = childrenArrayJsonNode.addObject();
         if (child.isJSONRoot())
         {
            childJsonNode.put("file", child.getJSONFilename());
            child.saveToFile();
         }
         else
         {
            child.saveToFile(childJsonNode);
         }
      }
   }

   /**
    * Loads just this node's definition data. Not recursive
    * because higher level node builders are required.
    */
   public void loadFromFile(JsonNode jsonNode)
   {
      description.setValue(jsonNode.get("description").textValue());
   }

   public void toMessage(BehaviorTreeNodeDefinitionMessage message)
   {
      message.setDescription(description.toMessage());
      message.setNumberOfChildren(children.size());
      message.setJsonFileName(jsonFileName.toMessage());
   }

   public void fromMessage(BehaviorTreeNodeDefinitionMessage message)
   {
      description.fromMessage(message.getDescriptionAsString());
      jsonFileName.fromMessage(message.getJsonFileNameAsString());
   }

   /**
    * A description of the action to help the operator in understanding
    * the purpose and context of the action.
    */
   public void setDescription(String description)
   {
      this.description.setValue(description);
   }

   public String getDescription()
   {
      return description.getValue();
   }

   public void setJSONFileName(String jsonFileName)
   {
      this.jsonFileName.setValue(jsonFileName);
   }

   public String getJSONFilename()
   {
      return jsonFileName.getValue();
   }

   public boolean isJSONRoot()
   {
      return !jsonFileName.getValue().isEmpty();
   }

   @Override
   public List<BehaviorTreeNodeDefinition> getChildren()
   {
      return children;
   }

   @Override
   public void setParent(@Nullable BehaviorTreeNodeDefinition parent)
   {
      this.parent = parent;
   }

   @Nullable
   @Override
   public BehaviorTreeNodeDefinition getParent()
   {
      return parent;
   }
}
