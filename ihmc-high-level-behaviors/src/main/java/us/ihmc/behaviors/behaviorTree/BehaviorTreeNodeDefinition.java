package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.BehaviorTreeNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
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
public class BehaviorTreeNodeDefinition extends LatestTimestampModifiable implements TreeNode<BehaviorTreeNodeDefinition>
{
   private final LatestTimestampModifiable childrenModification;
   /**
    * The name of the node.
    * It should be a set of words that summarize the node and that fits onto one line.
    * Always ends with .json if it's a JSON root node.
    * i.e. "PickUpObject.json"
    * i.e. "Move left hand"
    */
   private final CRDTBidirectionalString name;
   /**
    * Long form notes about the node.
    * Can be in paragraph form and include notes about the current
    * development state.
    */
   private final CRDTBidirectionalString notes;
   /** Behavior tree children node definitions. */
   private final List<BehaviorTreeNodeDefinition> children = new ArrayList<>();
   protected final BehaviorTreeRootNodeDefinition rootNode;
   private transient BehaviorTreeNodeDefinition parent;
   protected final WorkspaceResourceDirectory saveFileDirectory;
   protected final DRCRobotModel robotModel;

   // Used to compare with saved version and provide unsaved status (*) to the operator
   private String onDiskName;
   private String onDiskNotes;
   private final List<String> onDiskChildrenNames = new ArrayList<>();

   public BehaviorTreeNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      this(rootNode, rootNode.getCRDTInfo(), rootNode.getSaveFileDirectory(), rootNode.getRobotModel());
   }

   public BehaviorTreeNodeDefinition(BehaviorTreeRootNodeDefinition rootNode,
                                     CRDTInfo crdtInfo,
                                     WorkspaceResourceDirectory saveFileDirectory,
                                     DRCRobotModel robotModel)
   {
      super(crdtInfo);

      this.rootNode = rootNode == null ? (BehaviorTreeRootNodeDefinition) this : rootNode;
      this.saveFileDirectory = saveFileDirectory;
      this.robotModel = robotModel;

      childrenModification = new LatestTimestampModifiable(crdtInfo);
      name = new CRDTBidirectionalString(this, BehaviorTreeDefinitionRegistry.getInitialName(getClass()));
      notes = new CRDTBidirectionalString(this, "");

      updateName();
   }

   public void updateName()
   {
      setModifierName(name.getValue());
      childrenModification.setModifierName(name.getValue() + " children");
   }

   /** Save as JSON file root node. */
   public void saveToFile()
   {
      if (isJSONRoot())
      {
         WorkspaceResourceFile saveFile = new WorkspaceResourceFile(saveFileDirectory, getName());
         LogTools.info("Saving behavior tree: {}", saveFile.getFilesystemFile());
         if (JSONFileTools.save(saveFile, this::saveToFile)) // Success
         {
            BehaviorTreeTools.runForSubtreeNodes(this, BehaviorTreeNodeDefinition::setOnDiskFields);
         }
      }
      else
      {
         for (BehaviorTreeNodeDefinition child : children)
         {
            child.saveToFile();
         }
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

   /**
    * Loads just this node's definition data. Not recursive
    * because higher level node builders are required.
    */
   public void loadFromFile(JsonNode jsonNode)
   {
      name.setValue(jsonNode.get("name").textValue());
      notes.setValue(jsonNode.get("notes").textValue());

      updateName();
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
      if (isUndoAvailable())
      {
         name.setValue(onDiskName);
         notes.setValue(onDiskNotes);

         updateName();
      }

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
      message.setType(BehaviorTreeDefinitionRegistry.getMessageByte(getClass()));

      toMessage(message.getLatestModificationToData());
      childrenModification.toMessage(message.getLatestModificationToChildren());

      message.setName(name.toMessage());
      // message.setNotes(notes.toMessage());
      message.setNumberOfChildren((short) children.size());
   }

   public void fromMessage(BehaviorTreeNodeDefinitionMessage message)
   {
      fromMessage(message, false);
   }

   public void fromMessage(BehaviorTreeNodeDefinitionMessage message, boolean checkOnly)
   {
      // Needs to be done first to detect incoming modification
      fromMessage(message.getLatestModificationToData(), checkOnly);
      childrenModification.fromMessage(message.getLatestModificationToChildren());

      name.fromMessage(message.getNameAsString());
      // notes.fromMessage(message.getNotesAsString());

      updateName();
   }

   public LatestTimestampModifiable getChildrenModification()
   {
      return childrenModification;
   }

   public boolean isUndoAvailable()
   {
      return onDiskName != null;
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
