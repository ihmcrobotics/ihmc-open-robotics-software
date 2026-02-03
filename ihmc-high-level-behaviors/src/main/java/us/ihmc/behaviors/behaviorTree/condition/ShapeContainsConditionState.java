package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTStatusBoolean;
import us.ihmc.communication.crdt.CRDTStatusInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class ShapeContainsConditionState
{
   private final CRDTStatusInteger numberOfPointsContained;
   private final CRDTStatusBoolean frameIsContained;
   private final CRDTDetachableReferenceFrame shapeFrame;

   public ShapeContainsConditionState(ConditionNodeDefinition definition, BehaviorTreeSceneState scene)
   {
      numberOfPointsContained = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), 0);
      frameIsContained = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), false);
      shapeFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName,
                                                    definition.getShapeContains().getShapeParentFrameNameCRDT(),
                                                    definition.getShapeContains().getShapeTransformToParent());
   }

   public void update()
   {
      shapeFrame.update();
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      message.setNumberOfPointsContained(numberOfPointsContained.toMessage());
      message.setFrameIsContained(frameIsContained.toMessage());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      numberOfPointsContained.fromMessage((int) message.getNumberOfPointsContained());
      frameIsContained.fromMessage(message.getFrameIsContained());
   }

   public CRDTDetachableReferenceFrame getShapeFrame()
   {
      return shapeFrame;
   }

   public int getNumberOfPointsContained()
   {
      return numberOfPointsContained.getValue();
   }

   public void setNumberOfPointsContained(int value)
   {
      numberOfPointsContained.setValue(value);
   }

   public boolean getFrameIsContained()
   {
      return frameIsContained.getValue();
   }

   public void setFrameIsContained(boolean value)
   {
      frameIsContained.setValue(value);
   }
}
