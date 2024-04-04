package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KickDoorActionDefinitionMessage extends Packet<KickDoorActionDefinitionMessage> implements Settable<KickDoorActionDefinitionMessage>, EpsilonComparable<KickDoorActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Specifies the side of the robot that will execute the kick.
            */
   public byte robot_side_ = (byte) 255;
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
            * Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
            */
   public double prekick_weight_distribution_;

   public KickDoorActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
   }

   public KickDoorActionDefinitionMessage(KickDoorActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(KickDoorActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      kick_height_ = other.kick_height_;

      kick_impulse_ = other.kick_impulse_;

      kick_target_distance_ = other.kick_target_distance_;

      prekick_weight_distribution_ = other.prekick_weight_distribution_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
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


   public static Supplier<KickDoorActionDefinitionMessagePubSubType> getPubSubType()
   {
      return KickDoorActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KickDoorActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KickDoorActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_height_, other.kick_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_impulse_, other.kick_impulse_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_target_distance_, other.kick_target_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.prekick_weight_distribution_, other.prekick_weight_distribution_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KickDoorActionDefinitionMessage)) return false;

      KickDoorActionDefinitionMessage otherMyClass = (KickDoorActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.kick_height_ != otherMyClass.kick_height_) return false;

      if(this.kick_impulse_ != otherMyClass.kick_impulse_) return false;

      if(this.kick_target_distance_ != otherMyClass.kick_target_distance_) return false;

      if(this.prekick_weight_distribution_ != otherMyClass.prekick_weight_distribution_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KickDoorActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("kick_height=");
      builder.append(this.kick_height_);      builder.append(", ");
      builder.append("kick_impulse=");
      builder.append(this.kick_impulse_);      builder.append(", ");
      builder.append("kick_target_distance=");
      builder.append(this.kick_target_distance_);      builder.append(", ");
      builder.append("prekick_weight_distribution=");
      builder.append(this.prekick_weight_distribution_);
      builder.append("}");
      return builder.toString();
   }
}
