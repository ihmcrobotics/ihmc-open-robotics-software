package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.logic.condition.CounterConditionDefinition;
import us.ihmc.behaviors.sequence.LeafNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * The first version of this node just implements a counter.
 *
 * TODO: Extend such that conditions can be setup through an interface
 *   and registered as an option for the operator to select.
 */
public class ConditionNodeDefinition extends LeafNodeDefinition
{
   public enum Type
   {
      COUNTER,
      LLM;

      public static final Type[] values = values();
   }

   private final CRDTBidirectionalEnumField<Type> type;

   private final CounterConditionDefinition counter;

   private Type onDiskType;

   public ConditionNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      type = new CRDTBidirectionalEnumField<>(this, Type.COUNTER);

      counter = new CounterConditionDefinition(this);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("conditionType", type.getValue().name());

      switch (type.getValue())
      {
         case COUNTER -> counter.saveToFile(jsonNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      type.setValue(Type.valueOf(jsonNode.get("conditionType").textValue()));

      switch (type.getValue())
      {
         case COUNTER -> counter.loadFromFile(jsonNode);
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskType = type.getValue();

      switch (type.getValue())
      {
         case COUNTER -> counter.setOnDiskFields();
      }
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         type.setValue(onDiskType);

         switch (type.getValue())
         {
            case COUNTER -> counter.undoAllNontopologicalChanges();
         }
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= type.getValue() == onDiskType;

      switch (type.getValue())
      {
         case COUNTER -> unchanged &= !counter.hasChanges();
      }

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setType((byte) type.toMessage().ordinal());

      switch (type.getValue())
      {
         case COUNTER -> counter.toMessage(message);
      }
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      type.fromMessage(Type.values()[message.getType()]);

      switch (type.getValue())
      {
         case COUNTER -> counter.fromMessage(message);
      }
   }

   public CRDTBidirectionalEnumField<Type> getType()
   {
      return type;
   }

   public CounterConditionDefinition getCounter()
   {
      return counter;
   }
}
