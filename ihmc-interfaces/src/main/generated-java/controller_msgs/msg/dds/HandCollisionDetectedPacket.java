package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Message for the IHMC humanoid behavior module.
 */
public class HandCollisionDetectedPacket implements Settable<HandCollisionDetectedPacket>, EpsilonComparable<HandCollisionDetectedPacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   private byte robot_side_ = (byte) 255;
   private int collision_severity_level_one_to_three_;

   public HandCollisionDetectedPacket()
   {

   }

   public HandCollisionDetectedPacket(HandCollisionDetectedPacket other)
   {
      set(other);
   }

   public void set(HandCollisionDetectedPacket other)
   {
      robot_side_ = other.robot_side_;

      collision_severity_level_one_to_three_ = other.collision_severity_level_one_to_three_;
   }

   public byte getRobotSide()
   {
      return robot_side_;
   }

   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   public int getCollisionSeverityLevelOneToThree()
   {
      return collision_severity_level_one_to_three_;
   }

   public void setCollisionSeverityLevelOneToThree(int collision_severity_level_one_to_three)
   {
      collision_severity_level_one_to_three_ = collision_severity_level_one_to_three;
   }

   @Override
   public boolean epsilonEquals(HandCollisionDetectedPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_severity_level_one_to_three_, other.collision_severity_level_one_to_three_, epsilon))
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
      if (!(other instanceof HandCollisionDetectedPacket))
         return false;

      HandCollisionDetectedPacket otherMyClass = (HandCollisionDetectedPacket) other;

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      if (this.collision_severity_level_one_to_three_ != otherMyClass.collision_severity_level_one_to_three_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandCollisionDetectedPacket {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);

      builder.append(", ");
      builder.append("collision_severity_level_one_to_three=");
      builder.append(this.collision_severity_level_one_to_three_);

      builder.append("}");
      return builder.toString();
   }
}