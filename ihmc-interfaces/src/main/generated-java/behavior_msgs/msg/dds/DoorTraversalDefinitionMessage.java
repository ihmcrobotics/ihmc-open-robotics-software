package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DoorTraversalDefinitionMessage extends Packet<DoorTraversalDefinitionMessage> implements Settable<DoorTraversalDefinitionMessage>, EpsilonComparable<DoorTraversalDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * The hand open angle to assume we lost the grasp on the object
            */
   public double lost_grasp_detection_hand_open_angle_;
   public double opened_door_handle_distance_from_start_;

   public DoorTraversalDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public DoorTraversalDefinitionMessage(DoorTraversalDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorTraversalDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      lost_grasp_detection_hand_open_angle_ = other.lost_grasp_detection_hand_open_angle_;

      opened_door_handle_distance_from_start_ = other.opened_door_handle_distance_from_start_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The hand open angle to assume we lost the grasp on the object
            */
   public void setLostGraspDetectionHandOpenAngle(double lost_grasp_detection_hand_open_angle)
   {
      lost_grasp_detection_hand_open_angle_ = lost_grasp_detection_hand_open_angle;
   }
   /**
            * The hand open angle to assume we lost the grasp on the object
            */
   public double getLostGraspDetectionHandOpenAngle()
   {
      return lost_grasp_detection_hand_open_angle_;
   }

   public void setOpenedDoorHandleDistanceFromStart(double opened_door_handle_distance_from_start)
   {
      opened_door_handle_distance_from_start_ = opened_door_handle_distance_from_start;
   }
   public double getOpenedDoorHandleDistanceFromStart()
   {
      return opened_door_handle_distance_from_start_;
   }


   public static Supplier<DoorTraversalDefinitionMessagePubSubType> getPubSubType()
   {
      return DoorTraversalDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DoorTraversalDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DoorTraversalDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lost_grasp_detection_hand_open_angle_, other.lost_grasp_detection_hand_open_angle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.opened_door_handle_distance_from_start_, other.opened_door_handle_distance_from_start_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DoorTraversalDefinitionMessage)) return false;

      DoorTraversalDefinitionMessage otherMyClass = (DoorTraversalDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.lost_grasp_detection_hand_open_angle_ != otherMyClass.lost_grasp_detection_hand_open_angle_) return false;

      if(this.opened_door_handle_distance_from_start_ != otherMyClass.opened_door_handle_distance_from_start_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorTraversalDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("lost_grasp_detection_hand_open_angle=");
      builder.append(this.lost_grasp_detection_hand_open_angle_);      builder.append(", ");
      builder.append("opened_door_handle_distance_from_start=");
      builder.append(this.opened_door_handle_distance_from_start_);
      builder.append("}");
      return builder.toString();
   }
}
