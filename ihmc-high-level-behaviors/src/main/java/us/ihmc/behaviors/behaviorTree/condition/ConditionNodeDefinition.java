package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.LeafNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;

/**
 * The first version of this node just implements a counter.
 *
 * TODO: Extend such that conditions can be setup through an interface
 *   and registered as an option for the operator to select.
 */
public class ConditionNodeDefinition extends LeafNodeDefinition
{
   public enum ConditionNodeType
   {
      COUNTER,
      LLM,
      PROXIMITY,
      ALWAYS_FAIL,
      ALWAYS_SUCCEED;

      public static final ConditionNodeType[] values = values();
   }

   private final CRDTBidirectionalEnumField<ConditionNodeType> conditionType;

   private final CounterConditionDefinition counter;
   private final LLMConditionDefinition llm;
   private final ProximityConditionDefinition proximityCheck;

   private ConditionNodeType onDiskConditionType;

   public ConditionNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      conditionType = new CRDTBidirectionalEnumField<>(this, ConditionNodeType.COUNTER);

      // TODO: Do we create them all or only as needed?
      counter = new CounterConditionDefinition(this);
      llm = new LLMConditionDefinition(this);
      proximityCheck = new ProximityConditionDefinition(this);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("conditionType", conditionType.getValue().name());

      switch (conditionType.getValue())
      {
         case COUNTER -> counter.saveToFile(jsonNode);
         case LLM -> llm.saveToFile(jsonNode);
         case PROXIMITY -> proximityCheck.saveToFile(jsonNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      conditionType.setValue(ConditionNodeType.valueOf(jsonNode.get("conditionType").textValue()));

      switch (conditionType.getValue())
      {
         case COUNTER -> counter.loadFromFile(jsonNode);
         case LLM -> llm.loadFromFile(jsonNode);
         case PROXIMITY -> proximityCheck.loadFromFile(jsonNode);
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskConditionType = conditionType.getValue();

      switch (conditionType.getValue())
      {
         case COUNTER -> counter.setOnDiskFields();
         case LLM -> llm.setOnDiskFields();
         case PROXIMITY -> proximityCheck.setOnDiskFields();
      }
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         conditionType.setValue(onDiskConditionType);

         switch (conditionType.getValue())
         {
            case COUNTER -> counter.undoAllNontopologicalChanges();
            case LLM -> llm.undoAllNontopologicalChanges();
            case PROXIMITY -> proximityCheck.undoAllNontopologicalChanges();
         }
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= conditionType.getValue() == onDiskConditionType;

      switch (conditionType.getValue())
      {
         case COUNTER -> unchanged &= !counter.hasChanges();
         case LLM -> unchanged &= !llm.hasChanges();
         case PROXIMITY -> unchanged &= !proximityCheck.hasChanges();
      }

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setType((byte) conditionType.toMessage().ordinal());

      switch (conditionType.getValue())
      {
         case COUNTER -> counter.toMessage(message);
         case LLM -> llm.toMessage(message);
         case PROXIMITY -> proximityCheck.toMessage(message);
      }
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      conditionType.fromMessage(ConditionNodeType.values()[message.getType()]);

      switch (conditionType.getValue())
      {
         case COUNTER -> counter.fromMessage(message);
         case LLM -> llm.fromMessage(message);
         case PROXIMITY -> proximityCheck.fromMessage(message);
      }
   }

   public CRDTBidirectionalEnumField<ConditionNodeType> getConditionType()
   {
      return conditionType;
   }

   public CounterConditionDefinition getCounter()
   {
      return counter;
   }

   public LLMConditionDefinition getLLM()
   {
      return llm;
   }

   public ProximityConditionDefinition getProximityCheck()
   {
      return proximityCheck;
   }
}