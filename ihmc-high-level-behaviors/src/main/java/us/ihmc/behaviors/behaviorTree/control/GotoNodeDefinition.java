package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.GotoNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.LeafNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;

public class GotoNodeDefinition extends LeafNodeDefinition
{
   public static final long GOTO_INVALID_ID = GotoNodeDefinitionMessage.INVALID;
   public static final long GOTO_NEXT_ID = GotoNodeDefinitionMessage.GOTO_NEXT;
   public static final String GOTO_NEXT_NAME = "Next";

   private final CRDTBidirectionalLong nodeToGotoID;
   /** We use this to save the node to goto name to file instead of the number for human readability. */
   private String nodeToGotoName = GOTO_NEXT_NAME;

   // On disk fields
   private String onDiskNodeToGotoName;

   public GotoNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      nodeToGotoID = new CRDTBidirectionalLong(this, GOTO_NEXT_ID);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("nodeToGoto", nodeToGotoName);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      setNodeToGotoNameUnknownID(jsonNode.get("nodeToGoto").textValue());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskNodeToGotoName = nodeToGotoName;
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         setNodeToGotoNameUnknownID(onDiskNodeToGotoName);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= nodeToGotoName.equals(onDiskNodeToGotoName);

      return !unchanged;
   }

   public void toMessage(GotoNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setNodeToGotoId(nodeToGotoID.toMessage());
   }

   public void fromMessage(GotoNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      nodeToGotoID.fromMessage(message.getNodeToGotoId());
   }

   public boolean getNodeToGotoIsInvalid()
   {
      return nodeToGotoID.getValue() == GOTO_INVALID_ID;
   }

   public boolean getGotoNextNode()
   {
      return nodeToGotoID.getValue() == GOTO_NEXT_ID;
   }

   private void setNodeToGotoNameUnknownID(String name)
   {
      switch (name)
      {
         case GOTO_NEXT_NAME -> setGotoNextNode();
         default -> setNodeToGoto(GOTO_INVALID_ID, name);
      }
   }

   public void setNodeToGoto(long id, String name)
   {
      nodeToGotoID.setValue(id);
      nodeToGotoName = name;
   }

   public void setGotoNextNode()
   {
      nodeToGotoID.setValue(GOTO_NEXT_ID);
      nodeToGotoName = GOTO_NEXT_NAME;
   }

   public long getNodeToGotoID()
   {
      return nodeToGotoID.getValue();
   }

   public void setNodeToGotoName(String nodeToGotoName)
   {
      this.nodeToGotoName = nodeToGotoName;
   }

   /** Only used for finding the ID after loading */
   public String getNodeToGotoName()
   {
      return nodeToGotoName;
   }
}
