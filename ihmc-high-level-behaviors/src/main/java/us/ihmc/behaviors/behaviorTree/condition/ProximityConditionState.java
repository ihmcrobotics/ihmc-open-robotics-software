package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.ConditionNodeStateMessage;
import us.ihmc.communication.crdt.CRDTStatusBoolean;
import us.ihmc.communication.crdt.CRDTStatusVector3D;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class ProximityConditionState
{
   private final CRDTStatusVector3D vectorBToA;
   private final CRDTStatusBoolean frameAIsPresent;
   private final CRDTStatusBoolean frameBIsPresent;

   public ProximityConditionState(ConditionNodeDefinition definition)
   {
      vectorBToA = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo());
      frameAIsPresent = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), false);
      frameBIsPresent = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, definition.getCRDTInfo(), false);
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      message.getBToA().set(vectorBToA.getValueReadOnly());
      message.setFrameAIsPresent(frameAIsPresent.toMessage());
      message.setFrameBIsPresent(frameBIsPresent.toMessage());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      vectorBToA.fromMessage(message.getBToA().getVector());
      frameAIsPresent.fromMessage(message.getFrameAIsPresent());
      frameBIsPresent.fromMessage(message.getFrameBIsPresent());
   }

   public Vector3DReadOnly getVectorBToA()
   {
      return vectorBToA.getValueReadOnly();
   }

   public void setVectorBToA(Vector3DReadOnly value, double epsilon)
   {
      vectorBToA.setValue(value, epsilon);
   }

   public boolean getFrameAIsPresent()
   {
      return frameAIsPresent.getValue();
   }

   public void setFrameAIsPresent(boolean value)
   {
      frameAIsPresent.setValue(value);
   }

   public boolean getFrameBIsPresent()
   {
      return frameBIsPresent.getValue();
   }

   public void setFrameBIsPresent(boolean value)
   {
      frameBIsPresent.setValue(value);
   }
}