package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusBoolean;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeState extends LeafNodeState<ConditionNodeDefinition>
{
   private final CounterConditionState counter;
   private final LLMConditionState llm;
   private final ProximityConditionState proximityCheck;
   private final CRDTStatusBoolean conditionMet;
   private final CRDTStatusBoolean evaluatingCondition;

   public ConditionNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new ConditionNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      counter = new CounterConditionState(definition);
      llm = new LLMConditionState(definition);
      proximityCheck = new ProximityConditionState(definition);

      conditionMet = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      evaluatingCondition = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      switch (definition.getType().getValue())
      {
         case COUNTER -> counter.toMessage(message);
         case LLM -> llm.toMessage(message);
         case PROXIMITY -> proximityCheck.toMessage(message);
      }
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      switch (definition.getType().getValue())
      {
         case COUNTER -> counter.fromMessage(message);
         case LLM -> llm.fromMessage(message);
         case PROXIMITY -> proximityCheck.fromMessage(message);
      }
   }

   public CounterConditionState getCounter()
   {
      return counter;
   }

   public LLMConditionState getLLM()
   {
      return llm;
   }

   public ProximityConditionState getProximityCheck()
   {
      return proximityCheck;
   }

   public void setConditionValue(boolean value)
   {
      conditionMet.setValue(value);
   }

   public boolean isConditionMet()
   {
      return conditionMet.getValue();
   }

   public void setEvaluatingConditionValue(boolean value)
   {
      evaluatingCondition.setValue(value);
   }

   public boolean isEvaluatingCondition()
   {
      return evaluatingCondition.getValue();
   }
}