package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part the IHMC humanoid behavior module.
 */
public class ObjectWeightPacket extends Packet<ObjectWeightPacket> implements Settable<ObjectWeightPacket>, EpsilonComparable<ObjectWeightPacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public byte robot_side_ = (byte) 255;
   public double weight_;

   public ObjectWeightPacket()
   {

   }

   public ObjectWeightPacket(ObjectWeightPacket other)
   {
      set(other);
   }

   public void set(ObjectWeightPacket other)
   {
      robot_side_ = other.robot_side_;

      weight_ = other.weight_;
   }

   public byte getRobotSide()
   {
      return robot_side_;
   }

   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   public double getWeight()
   {
      return weight_;
   }

   public void setWeight(double weight)
   {
      weight_ = weight;
   }

   @Override
   public boolean epsilonEquals(ObjectWeightPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.weight_, other.weight_, epsilon))
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
      if (!(other instanceof ObjectWeightPacket))
         return false;

      ObjectWeightPacket otherMyClass = (ObjectWeightPacket) other;

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      if (this.weight_ != otherMyClass.weight_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ObjectWeightPacket {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);

      builder.append(", ");
      builder.append("weight=");
      builder.append(this.weight_);

      builder.append("}");
      return builder.toString();
   }
}