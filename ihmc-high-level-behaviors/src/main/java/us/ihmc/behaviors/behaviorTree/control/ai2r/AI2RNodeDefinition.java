package us.ihmc.behaviors.behaviorTree.control.ai2r;

import behavior_msgs.AI2RNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;

public class AI2RNodeDefinition extends BehaviorTreeNodeDefinition
{
   public static final boolean DEFAULT_RANDOMIZE_GO_TO_ACTION = false;
   public static final int DEFAULT_NUMBER_OF_RANDOMIZATIONS = 1;

   private final CRDTBidirectionalBoolean randomizeGoToAction;
   private final CRDTBidirectionalInteger numberOfRandomizations;

   // On disk fields
   private boolean onDiskRandomizeGoToAction;
   private int onDiskNumberOfRandomizations;

   public AI2RNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      randomizeGoToAction = new CRDTBidirectionalBoolean(this, DEFAULT_RANDOMIZE_GO_TO_ACTION);
      numberOfRandomizations = new CRDTBidirectionalInteger(this, DEFAULT_NUMBER_OF_RANDOMIZATIONS);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("randomizeGoToAction", randomizeGoToAction.getValue());
      jsonNode.put("numberOfRandomizations", numberOfRandomizations.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      if (jsonNode.has("randomizeGoToAction"))
         setRandomizeGoToActionEnabled(jsonNode.get("randomizeGoToAction").asBoolean());
      if (jsonNode.has("numberOfRandomizations"))
         setNumberOfRandomizationsValue(jsonNode.get("numberOfRandomizations").asInt());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskRandomizeGoToAction = randomizeGoToAction.getValue();
      onDiskNumberOfRandomizations = numberOfRandomizations.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         setRandomizeGoToActionEnabled(onDiskRandomizeGoToAction);
         setNumberOfRandomizationsValue(onDiskNumberOfRandomizations);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= randomizeGoToAction.getValue() == onDiskRandomizeGoToAction;
      unchanged &= numberOfRandomizations.getValue() == onDiskNumberOfRandomizations;

      return !unchanged;
   }

   public void toMessage(AI2RNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
      message.setRandomizeGoToAction(randomizeGoToAction.toMessage());
      message.setNumberOfRandomizations(numberOfRandomizations.toMessage());
   }

   public void fromMessage(AI2RNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
      randomizeGoToAction.fromMessage(message.getRandomizeGoToAction());
      numberOfRandomizations.fromMessage(message.getNumberOfRandomizations());
      setNumberOfRandomizationsValue(numberOfRandomizations.getValue());
   }

   public boolean getRandomizeGoToActionEnabled()
   {
      return randomizeGoToAction.getValue();
   }

   public void setRandomizeGoToActionEnabled(boolean enabled)
   {
      randomizeGoToAction.setValue(enabled);
   }

   public int getNumberOfRandomizationsValue()
   {
      return numberOfRandomizations.getValue();
   }

   public void setNumberOfRandomizationsValue(int numberOfRandomizations)
   {
      this.numberOfRandomizations.setValue(Math.max(0, numberOfRandomizations));
   }

   public CRDTBidirectionalBoolean getRandomizeGoToAction()
   {
      return randomizeGoToAction;
   }

   public CRDTBidirectionalInteger getNumberOfRandomizations()
   {
      return numberOfRandomizations;
   }
}
