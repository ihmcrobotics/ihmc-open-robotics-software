package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalString;
import us.ihmc.communication.crdt.RequestConfirmFreezable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * The base definition of a behavior tree node.
 */
public class BehaviorTreeNodeDefinition extends RequestConfirmFreezable implements BehaviorTreeNode<BehaviorTreeNodeDefinition>
{
   /**
    * The name of the node.
    * It should be a set of words that summarize the node and that fits onto one line.
    * Always ends with .json if it's a JSON root node.
    * i.e. "PickUpObject.json"
    * i.e. "Move left hand"
    */
   private final CRDTUnidirectionalString name;
   /**
    * Long form notes about the node.
    * Can be in paragraph form and include notes about the current
    * development state.
    */
   private final CRDTUnidirectionalString notes;
   /** Behavior tree children node definitions. */
   private final List<BehaviorTreeNodeDefinition> children = new ArrayList<>();
   private transient BehaviorTreeNodeDefinition parent;
   private final WorkspaceResourceDirectory saveFileDirectory;

   // Used to compare with saved version and provide unsaved status (*) to the operator
   private String onDiskName;
   private String onDiskNotes;
   private final List<String> onDiskChildrenNames = new ArrayList<>();

   public BehaviorTreeNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo);

      this.saveFileDirectory = saveFileDirectory;

      name = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, this, "");
      notes = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, this, "");
   }

   /** Save as JSON file root node. */
   public void saveToFile()
   {
      if (!isJSONRoot())
         LogTools.error("Cannot save. Can only be called for JSON roots.");

      WorkspaceResourceFile saveFile = new WorkspaceResourceFile(saveFileDirectory, name.getValue());
      LogTools.info("Saving behavior tree: {}", saveFile.getFilesystemFile());
      if (JSONFileTools.save(saveFile, this::saveToFile)) // Success
      {
         BehaviorTreeTools.runForSubtreeNodes(this, BehaviorTreeNodeDefinition::setOnDiskFields);
      }
   }

   /**
    * Saves the file recursively.
    */
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("type", getClass().getSimpleName());
      jsonNode.put("name", name.getValue());
      jsonNode.put("notes", notes.getValue());

      ArrayNode childrenArrayJsonNode = jsonNode.putArray("children");
      for (BehaviorTreeNodeDefinition child : children)
      {
         ObjectNode childJsonNode = childrenArrayJsonNode.addObject();
         if (child.isJSONRoot())
         {
            childJsonNode.put("file", child.getName());
            child.saveToFile();
         }
         else
         {
            child.saveToFile(childJsonNode);
         }
      }
   }

   public void drawToSVG()
   {
      new BehaviorTreeSVGWriter(this, saveFileDirectory);
   }

   /**
    * Loads just this node's definition data. Not recursive
    * because higher level node builders are required.
    */
   public void loadFromFile(JsonNode jsonNode)
   {
      name.setValue(jsonNode.get("name").textValue());
      notes.setValue(jsonNode.get("notes").textValue());
   }

   public void setOnDiskFields()
   {
      onDiskName = name.getValue();
      onDiskNotes = notes.getValue();

      onDiskChildrenNames.clear();
      for (BehaviorTreeNodeDefinition child : children)
         onDiskChildrenNames.add(child.getName());
   }

   public void undoAllNontopologicalChanges()
   {
      name.setValue(onDiskName);
      notes.setValue(onDiskNotes);

      // We are not able to undo changes to children topology.
      // The user must delete and reload the entire tree.

      for (BehaviorTreeNodeDefinition child : children)
      {
         child.undoAllNontopologicalChanges();
      }
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;
      unchanged &= name.getValue().equals(onDiskName);
      unchanged &= notes.getValue().equals(onDiskNotes);

      boolean childrenSizeEquals = onDiskChildrenNames.size() == children.size();
      unchanged &= childrenSizeEquals;
      if (childrenSizeEquals)
         for (int i = 0; i < children.size(); i++)
            unchanged &= children.get(i).getName().equals(onDiskChildrenNames.get(i));

      return !unchanged;
   }

   public void toMessage(BehaviorTreeNodeDefinitionMessage message)
   {
      toMessage(message.getConfirmableRequest());

      message.setName(name.toMessage());
      // message.setNotes(notes.toMessage());
      message.setNumberOfChildren(children.size());
   }

   public void fromMessage(BehaviorTreeNodeDefinitionMessage message)
   {
      fromMessage(message.getConfirmableRequest()); // Unpack first, because this also unfreezes

      name.fromMessage(message.getNameAsString());
      // notes.fromMessage(message.getNotesAsString());
   }

   public void setName(String name)
   {
      this.name.setValue(name);
   }

   public String getName()
   {
      return name.getValue();
   }

   public void setNotes(String notes)
   {
      this.notes.setValue(notes);
   }

   public String getNotes()
   {
      return notes.getValue();
   }

   public boolean isJSONRoot()
   {
      return name.getValue().endsWith(".json");
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
