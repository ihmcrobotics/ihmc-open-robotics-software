package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.LeafNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

/**
 * Definition of a leaf node which gets executed as part of the concurrency system.
 *
 * This is used for action, condition, and goto nodes.
 */
public class LeafNodeDefinition extends BehaviorTreeNodeDefinition
{
   public static final String EXECUTE_AFTER_PREVIOUS = "Previous";
   public static final String EXECUTE_AFTER_BEGINNING = "Beginning";

   private final CRDTBidirectionalBoolean executeAfterPrevious;
   private final CRDTBidirectionalBoolean executeAfterBeginning;
   private final CRDTBidirectionalLong executeAfterNodeID;
   /** We use this to save the action name to file instead of the number for human readability. */
   private String executeAfterLeafName = EXECUTE_AFTER_PREVIOUS;

   // On disk fields
   private String onDiskExecuteAfterLeafName;

   public LeafNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      executeAfterPrevious = new CRDTBidirectionalBoolean(this, true);
      executeAfterBeginning = new CRDTBidirectionalBoolean(this, false);
      executeAfterNodeID = new CRDTBidirectionalLong(this, 0);
   }

   public void updateAndSanitizeExecuteAfterFields(@Nullable String executeAfterLeafName)
   {
      if (executeAfterBeginning.getValue())
      {
         this.executeAfterLeafName = EXECUTE_AFTER_BEGINNING;
         executeAfterNodeID.setValue(0);
      }
      else if (executeAfterLeafName != null)
      {
         this.executeAfterLeafName = executeAfterLeafName;
      }
      else // Default to previous
      {
         executeAfterPrevious.setValue(true);
         this.executeAfterLeafName = EXECUTE_AFTER_PREVIOUS;
         executeAfterNodeID.setValue(0);
      }
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("executeAfterAction", executeAfterLeafName);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      executeAfterLeafName = jsonNode.get("executeAfterAction").textValue();
      executeAfterPrevious.setValue(executeAfterLeafName.equals(EXECUTE_AFTER_PREVIOUS));
      executeAfterBeginning.setValue(executeAfterLeafName.equals(EXECUTE_AFTER_BEGINNING));
      executeAfterNodeID.setValue(0); // Invalidate until we can find it
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskExecuteAfterLeafName = executeAfterLeafName;
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         executeAfterLeafName = onDiskExecuteAfterLeafName;
         executeAfterPrevious.setValue(onDiskExecuteAfterLeafName.equals(EXECUTE_AFTER_PREVIOUS));
         executeAfterBeginning.setValue(onDiskExecuteAfterLeafName.equals(EXECUTE_AFTER_BEGINNING));
         executeAfterNodeID.setValue(0); // Invalidate until we can find it
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= executeAfterLeafName.equals(onDiskExecuteAfterLeafName);

      return !unchanged;
   }

   public void toMessage(LeafNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setExecuteAfterPrevious(executeAfterPrevious.toMessage());
      message.setExecuteAfterBeginning(executeAfterBeginning.toMessage());
      message.setExecuteAfterNodeId(executeAfterNodeID.toMessage());
   }

   public void fromMessage(LeafNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      executeAfterPrevious.fromMessage(message.getExecuteAfterPrevious());
      executeAfterBeginning.fromMessage(message.getExecuteAfterBeginning());
      executeAfterNodeID.fromMessage(message.getExecuteAfterNodeId());
   }

   public CRDTBidirectionalBoolean getExecuteAfterPrevious()
   {
      return executeAfterPrevious;
   }

   public CRDTBidirectionalBoolean getExecuteAfterBeginning()
   {
      return executeAfterBeginning;
   }

   public CRDTBidirectionalLong getExecuteAfterNodeID()
   {
      return executeAfterNodeID;
   }

   /** Only used for finding the ID after loading */
   public String getExecuteAfterLeafName()
   {
      return executeAfterLeafName;
   }
}
