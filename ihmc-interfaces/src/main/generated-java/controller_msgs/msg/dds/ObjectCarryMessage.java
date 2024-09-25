package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ObjectCarryMessage extends Packet<ObjectCarryMessage> implements Settable<ObjectCarryMessage>, EpsilonComparable<ObjectCarryMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Specifies the side of the robot carrying the object
            */
   public byte robot_side_ = (byte) 255;
   /**
            * If true, will increase mass in the hand
            */
   public boolean is_picking_up_;
   /**
            * Mass of the object being picked up
            */
   public double object_mass_;

   public ObjectCarryMessage()
   {
   }

   public ObjectCarryMessage(ObjectCarryMessage other)
   {
      this();
      set(other);
   }

   public void set(ObjectCarryMessage other)
   {
      robot_side_ = other.robot_side_;

      is_picking_up_ = other.is_picking_up_;

      object_mass_ = other.object_mass_;

   }

   /**
            * Specifies the side of the robot carrying the object
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot carrying the object
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * If true, will increase mass in the hand
            */
   public void setIsPickingUp(boolean is_picking_up)
   {
      is_picking_up_ = is_picking_up;
   }
   /**
            * If true, will increase mass in the hand
            */
   public boolean getIsPickingUp()
   {
      return is_picking_up_;
   }

   /**
            * Mass of the object being picked up
            */
   public void setObjectMass(double object_mass)
   {
      object_mass_ = object_mass;
   }
   /**
            * Mass of the object being picked up
            */
   public double getObjectMass()
   {
      return object_mass_;
   }


   public static Supplier<ObjectCarryMessagePubSubType> getPubSubType()
   {
      return ObjectCarryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ObjectCarryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ObjectCarryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_picking_up_, other.is_picking_up_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.object_mass_, other.object_mass_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ObjectCarryMessage)) return false;

      ObjectCarryMessage otherMyClass = (ObjectCarryMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.is_picking_up_ != otherMyClass.is_picking_up_) return false;

      if(this.object_mass_ != otherMyClass.object_mass_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ObjectCarryMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("is_picking_up=");
      builder.append(this.is_picking_up_);      builder.append(", ");
      builder.append("object_mass=");
      builder.append(this.object_mass_);
      builder.append("}");
      return builder.toString();
   }
}
