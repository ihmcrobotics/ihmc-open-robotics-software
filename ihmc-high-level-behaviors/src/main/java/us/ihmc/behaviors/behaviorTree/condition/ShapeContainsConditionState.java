package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.ConditionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTStatusBoolean;
import us.ihmc.communication.crdt.CRDTStatusFloat;
import us.ihmc.communication.crdt.CRDTStatusInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class ShapeContainsConditionState
{
   private final CRDTStatusBoolean frameIsContained;
   private final CRDTStatusInteger numberOfPointsContained;
   private final CRDTStatusFloat averageHue;
   private final CRDTStatusFloat averageSaturation;
   private final CRDTStatusFloat averageValue;
   private final CRDTDetachableReferenceFrame shapeFrame;

   public ShapeContainsConditionState(ConditionNodeDefinition definition, BehaviorTreeSceneState scene)
   {
      shapeFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName,
                                                    definition.getShapeContains().getShapeParentFrameNameCRDT(),
                                                    definition.getShapeContains().getShapeTransformToParent());
      frameIsContained = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), false);
      numberOfPointsContained = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), 0);
      averageHue = new CRDTStatusFloat(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), 0.0f);
      averageSaturation = new CRDTStatusFloat(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), 0.0f);
      averageValue = new CRDTStatusFloat(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), 0.0f);
   }

   public void update()
   {
      shapeFrame.update();
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      message.setFrameIsContained(frameIsContained.toMessage());
      message.setNumberOfPointsContained(numberOfPointsContained.toMessage());
      message.setAverageHue(averageHue.toMessage());
      message.setAverageSaturation(averageSaturation.toMessage());
      message.setAverageValue(averageValue.toMessage());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      frameIsContained.fromMessage(message.getFrameIsContained());
      numberOfPointsContained.fromMessage((int) message.getNumberOfPointsContained());
      averageHue.fromMessage(message.getAverageHue());
      averageSaturation.fromMessage(message.getAverageSaturation());
      averageValue.fromMessage(message.getAverageValue());
   }

   public CRDTDetachableReferenceFrame getShapeFrame()
   {
      return shapeFrame;
   }

   public boolean getFrameIsContained()
   {
      return frameIsContained.getValue();
   }

   public void setFrameIsContained(boolean value)
   {
      frameIsContained.setValue(value);
   }

   public int getNumberOfPointsContained()
   {
      return numberOfPointsContained.getValue();
   }

   public void setNumberOfPointsContained(int value)
   {
      numberOfPointsContained.setValue(value);
   }

   public float getAverageHue()
   {
      return averageHue.getValue();
   }

   public void setAverageHue(float averageHue)
   {
      this.averageHue.setValue(averageHue);
   }

   public float getAverageSaturation()
   {
      return averageSaturation.getValue();
   }

   public void setAverageSaturation(float averageSaturation)
   {
      this.averageSaturation.setValue(averageSaturation);
   }

   public float getAverageValue()
   {
      return averageValue.getValue();
   }

   public void setAverageValue(float averageValue)
   {
      this.averageValue.setValue(averageValue);
   }
}
