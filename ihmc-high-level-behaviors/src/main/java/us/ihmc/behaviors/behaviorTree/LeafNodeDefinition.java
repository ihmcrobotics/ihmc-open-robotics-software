package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.LeafNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;

/**
 * Definition of a leaf node which gets executed as part of the concurrency system.
 *
 * This is used for action, condition, and goto nodes.
 */
public class LeafNodeDefinition extends BehaviorTreeNodeDefinition
{
   public static final long EXECUTE_AFTER_INVALID_ID = LeafNodeDefinitionMessage.INVALID;
   public static final long EXECUTE_AFTER_PREVIOUS_ID = LeafNodeDefinitionMessage.EXECUTE_AFTER_PREVIOUS;
   public static final long EXECUTE_AFTER_BEGINNING_ID = LeafNodeDefinitionMessage.EXECUTE_AFTER_BEGINNING;
   public static final String EXECUTE_AFTER_PREVIOUS_NAME = "Previous";
   public static final String EXECUTE_AFTER_BEGINNING_NAME = "Beginning";

   private final CRDTBidirectionalLong executeAfterNodeID;
   /** We use this to save the action name to file instead of the number for human readability. */
   private String executeAfterLeafName = EXECUTE_AFTER_PREVIOUS_NAME;

   // On disk fields
   private String onDiskExecuteAfterLeafName;

   public LeafNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      executeAfterNodeID = new CRDTBidirectionalLong(this, EXECUTE_AFTER_PREVIOUS_ID);
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

      setExecuteAfterLeafUnknownID(jsonNode.get("executeAfterAction").textValue());
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
         setExecuteAfterLeafUnknownID(onDiskExecuteAfterLeafName);
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

      message.setExecuteAfterNodeId(executeAfterNodeID.toMessage());
   }

   public void fromMessage(LeafNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      executeAfterNodeID.fromMessage(message.getExecuteAfterNodeId());
   }

   public boolean getExecuteAfterIsInvalid()
   {
      return executeAfterNodeID.getValue() == EXECUTE_AFTER_INVALID_ID;
   }

   public boolean getExecuteAfterPrevious()
   {
      return executeAfterNodeID.getValue() == EXECUTE_AFTER_PREVIOUS_ID;
   }

   public boolean getExecuteAfterBeginning()
   {
      return executeAfterNodeID.getValue() == EXECUTE_AFTER_BEGINNING_ID;
   }

   private void setExecuteAfterLeafUnknownID(String name)
   {
      switch (name)
      {
         case EXECUTE_AFTER_PREVIOUS_NAME -> setExecuteAfterPrevious();
         case EXECUTE_AFTER_BEGINNING_NAME -> setExecuteAfterBeginning();
         default -> setExecuteAfterLeaf(EXECUTE_AFTER_INVALID_ID, name);
      }
   }

   public void setExecuteAfterLeaf(long id, String name)
   {
      executeAfterNodeID.setValue(id);
      executeAfterLeafName = name;
   }
   
   public void setExecuteAfterPrevious()
   {
      executeAfterNodeID.setValue(EXECUTE_AFTER_PREVIOUS_ID);
      executeAfterLeafName = EXECUTE_AFTER_PREVIOUS_NAME;
   }

   public void setExecuteAfterBeginning()
   {
      executeAfterNodeID.setValue(EXECUTE_AFTER_BEGINNING_ID);
      executeAfterLeafName = EXECUTE_AFTER_BEGINNING_NAME;
   }

   public long getExecuteAfterNodeID()
   {
      return executeAfterNodeID.getValue();
   }

   public void setExecuteAfterLeafName(String executeAfterLeafName)
   {
      this.executeAfterLeafName = executeAfterLeafName;
   }

   /** Only used for finding the ID after loading */
   public String getExecuteAfterLeafName()
   {
      return executeAfterLeafName;
   }
}
