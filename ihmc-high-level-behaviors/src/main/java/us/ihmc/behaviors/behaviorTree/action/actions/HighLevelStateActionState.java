package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.HighLevelStateActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.CRDTStatusString;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class HighLevelStateActionState extends ActionNodeState<HighLevelStateActionDefinition>
{
   /** The RL models the controller offers, one name per line. Filled on the robot side for the operator's model selection. */
   private final CRDTStatusString availableRLModels;

   public HighLevelStateActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new HighLevelStateActionDefinition(rootNode.getDefinition()), rootNode);

      availableRLModels = new CRDTStatusString(ROS2ActorDesignation.ROBOT, crdtInfo, "");
   }

   public void toMessage(HighLevelStateActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setAvailableRlModels(availableRLModels.toMessage());
   }

   public void fromMessage(HighLevelStateActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      availableRLModels.fromMessage(message.getAvailableRlModelsAsString());
   }

   public CRDTStatusString getAvailableRLModels()
   {
      return availableRLModels;
   }
}
