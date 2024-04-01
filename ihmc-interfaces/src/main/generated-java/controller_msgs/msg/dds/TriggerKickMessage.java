package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

import java.util.function.Supplier;

import us.ihmc.pubsub.TopicDataType;

/**
 * This message is part of the IHMC whole-body controller API.
 * This message is used to trigger a donkey (back) kick. The robot should be in an appropriate position to execute the kick.
 */
public class TriggerKickMessage extends Packet<TriggerKickMessage> implements Settable<TriggerKickMessage>, EpsilonComparable<TriggerKickMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * Unique ID used to identify this message, should preferably be consecutively increasing.
    */
   public long sequence_id_;
   /**
    * Specifies the side of the robot that will execute the kick.
    */
   public byte robot_side_;
   /**
    * The height at which the kick should be targeted.
    */
   public double kick_height_;
   /**
    * The impulse with which the kick should be executed.
    */
   public double kick_impulse_;
   /**
    * The target distance from the robot to where the kick should be aimed.
    */
   public double kick_target_distance_;
   /**
    * A boolean for tracking whether a kick has been requested.
    */
   public boolean trigger_kick_request_;
   /**
    * Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
    */
   public double prekick_weight_distribution_;

   public TriggerKickMessage()
   {
   }

   public TriggerKickMessage(TriggerKickMessage other)
   {
      this();
      set(other);
   }

   public void set(TriggerKickMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      kick_height_ = other.kick_height_;

      kick_impulse_ = other.kick_impulse_;

      kick_target_distance_ = other.kick_target_distance_;

      trigger_kick_request_ = other.trigger_kick_request_;

      prekick_weight_distribution_ = other.prekick_weight_distribution_;
   }

   /**
    * Unique ID used to identify this message, should preferably be consecutively increasing.
    */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }

   /**
    * Unique ID used to identify this message, should preferably be consecutively increasing.
    */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
    * Specifies the side of the robot that will execute the kick.
    */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   /**
    * Specifies the side of the robot that will execute the kick.
    */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
    * The height at which the kick should be targeted.
    */
   public void setKickHeight(double kick_height)
   {
      kick_height_ = kick_height;
   }

   /**
    * The height at which the kick should be targeted.
    */
   public double getKickHeight()
   {
      return kick_height_;
   }

   /**
    * The impulse with which the kick should be executed.
    */
   public void setKickImpulse(double kick_impulse)
   {
      kick_impulse_ = kick_impulse;
   }

   /**
    * The impulse with which the kick should be executed.
    */
   public double getKickImpulse()
   {
      return kick_impulse_;
   }

   /**
    * The target distance from the robot to where the kick should be aimed.
    */
   public void setKickTargetDistance(double kick_target_distance)
   {
      kick_target_distance_ = kick_target_distance;
   }

   /**
    * The target distance from the robot to where the kick should be aimed.
    */
   public double getKickTargetDistance()
   {
      return kick_target_distance_;
   }

   /**
    * A boolean for tracking whether a kick has been requested.
    */
   public void setTriggerKickRequest(boolean trigger_kick_request)
   {
      trigger_kick_request_ = trigger_kick_request;
   }

   /**
    * A boolean for tracking whether a kick has been requested.
    */
   public boolean getTriggerKickRequest()
   {
      return trigger_kick_request_;
   }

   /**
    * Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
    */
   public void setPrekickWeightDistribution(double prekick_weight_distribution)
   {
      prekick_weight_distribution_ = prekick_weight_distribution;
   }

   /**
    * Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
    */
   public double getPrekickWeightDistribution()
   {
      return prekick_weight_distribution_;
   }

   public static Supplier<TriggerKickMessagePubSubType> getPubSubType()
   {
      return TriggerKickMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TriggerKickMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TriggerKickMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_height_, other.kick_height_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_impulse_, other.kick_impulse_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_target_distance_, other.kick_target_distance_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.trigger_kick_request_, other.trigger_kick_request_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.prekick_weight_distribution_, other.prekick_weight_distribution_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof TriggerKickMessage))
         return false;

      TriggerKickMessage otherMyClass = (TriggerKickMessage) other;

      if (this.sequence_id_ != otherMyClass.sequence_id_)
         return false;

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      if (this.kick_height_ != otherMyClass.kick_height_)
         return false;

      if (this.kick_impulse_ != otherMyClass.kick_impulse_)
         return false;

      if (this.kick_target_distance_ != otherMyClass.kick_target_distance_)
         return false;

      if (this.trigger_kick_request_ != otherMyClass.trigger_kick_request_)
         return false;

      if (this.prekick_weight_distribution_ != otherMyClass.prekick_weight_distribution_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TriggerKickMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);
      builder.append(", ");
      builder.append("kick_height=");
      builder.append(this.kick_height_);
      builder.append(", ");
      builder.append("kick_impulse=");
      builder.append(this.kick_impulse_);
      builder.append(", ");
      builder.append("kick_target_distance=");
      builder.append(this.kick_target_distance_);
      builder.append(", ");
      builder.append("trigger_kick_request=");
      builder.append(this.trigger_kick_request_);
      builder.append(", ");
      builder.append("prekick_weight_distribution=");
      builder.append(this.prekick_weight_distribution_);
      builder.append("}");
      return builder.toString();
   }
}
