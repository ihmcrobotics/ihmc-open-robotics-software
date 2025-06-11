package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Message used to report the current joint angles for the fingers of the Ability gripper.
 */
public class AbilityHandStatusMessage extends Packet<AbilityHandStatusMessage>
      implements Settable<AbilityHandStatusMessage>, EpsilonComparable<AbilityHandStatusMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * Specifies the side of the robot of the hand being referred to
    */
   public byte robot_side_ = (byte) 255;

   public double current_position_index_;
   public double current_position_middle_;
   public double current_position_ring_;
   public double current_position_pinky_;
   public double current_position_thumb_rotator_;
   public double current_position_thumb_flexor_;

   public double current_velocity_index_;
   public double current_velocity_middle_;
   public double current_velocity_ring_;
   public double current_velocity_pinky_;
   public double current_velocity_thumb_rotator_;
   public double current_velocity_thumb_flexor_;

   public AbilityHandStatusMessage()
   {
   }

   public AbilityHandStatusMessage(AbilityHandStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandStatusMessage other)
   {
      robot_side_ = other.robot_side_;

      current_position_index_ = other.current_position_index_;
      current_position_middle_ = other.current_position_middle_;
      current_position_ring_ = other.current_position_ring_;
      current_position_pinky_ = other.current_position_pinky_;
      current_position_thumb_rotator_ = other.current_position_thumb_rotator_;
      current_position_thumb_flexor_ = other.current_position_thumb_flexor_;

      current_velocity_index_ = other.current_velocity_index_;
      current_velocity_middle_ = other.current_velocity_middle_;
      current_velocity_ring_ = other.current_velocity_ring_;
      current_velocity_pinky_ = other.current_velocity_pinky_;
      current_velocity_thumb_rotator_ = other.current_velocity_thumb_rotator_;
      current_velocity_thumb_flexor_ = other.current_velocity_thumb_flexor_;
   }

   /**
    * Specifies the side of the robot of the hand being referred to
    */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   /**
    * Specifies the side of the robot of the hand being referred to
    */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   public void setCurrent_position_index_(double index) {
      current_position_index_ = index;
   }


   public double getCurrent_position_index_()
   {
      return current_position_index_;
   }

   public void setCurrent_position_middle_(double middle)
   {
      current_position_middle_ = middle;
   }
   public double getCurrent_position_middle_()
   {
      return current_position_middle_;
   }
   public void setCurrent_position_ring_(double ring)
   {
      current_position_ring_ = ring;
   }
   public double getCurrent_position_ring_()
   {
      return current_position_ring_;
   }
   public void setCurrent_position_pinky_(double pinky)
   {
      current_position_pinky_ = pinky;
   }
   public double getCurrent_position_pinky_()
   {
      return current_position_pinky_;
   }

   public void setCurrent_position_thumb_rotator(double thumb_rotator)
   {
      current_position_thumb_rotator_ = thumb_rotator;
   }
   public double getCurrent_position_thumb_rotator_()
   {
      return current_position_thumb_rotator_;
   }

   public void setCurrent_position_thumb_flexor(double thumb_flexor)
   {
      current_position_thumb_flexor_ = thumb_flexor;
   }
   public double getCurrent_position_thumb_flexor_()
   {
      return current_position_thumb_flexor_;
   }

   public double getCurrent_velocity_index_()
   {
      return current_velocity_index_;
   }

   public double getCurrent_velocity_middle_()
   {
      return current_velocity_middle_;
   }

   public double getCurrent_velocity_ring_()
   {
      return current_velocity_ring_;
   }

   public double getCurrent_velocity_pinky_()
   {
      return current_velocity_pinky_;
   }

   public double getCurrent_velocity_thumb_rotator_()
   {
      return current_velocity_thumb_rotator_;
   }

   public double getCurrent_velocity_thumb_flexor_()
   {
      return current_velocity_thumb_flexor_;
   }

   @Override
   public boolean epsilonEquals(AbilityHandStatusMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_index_, other.current_position_index_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_middle_, other.current_position_middle_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_ring_, other.current_position_ring_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_pinky_, other.current_position_pinky_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_thumb_rotator_, other.current_position_thumb_rotator_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_thumb_flexor_, other.current_position_thumb_flexor_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_velocity_index_, other.current_velocity_index_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_velocity_middle_, other.current_velocity_middle_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_velocity_ring_, other.current_velocity_ring_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_velocity_pinky_, other.current_velocity_pinky_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_velocity_thumb_rotator_, other.current_velocity_thumb_rotator_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_velocity_thumb_flexor_, other.current_velocity_thumb_flexor_, epsilon))
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
      if (!(other instanceof AbilityHandStatusMessage))
         return false;

      AbilityHandStatusMessage otherMyClass = (AbilityHandStatusMessage) other;

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;
      if (this.current_position_index_ != otherMyClass.current_position_index_)
         return false;
      if (this.current_position_middle_ != otherMyClass.current_position_middle_)
         return false;
      if (this.current_position_ring_ != otherMyClass.current_position_ring_)
         return false;
      if (this.current_position_pinky_ != otherMyClass.current_position_pinky_)
         return false;
      if (this.current_position_thumb_rotator_ != otherMyClass.current_position_thumb_rotator_)
         return false;
      if (this.current_position_thumb_flexor_ != otherMyClass.current_position_thumb_flexor_)
         return false;
      if (this.current_velocity_index_ != otherMyClass.current_velocity_index_)
         return false;
      if (this.current_velocity_middle_ != otherMyClass.current_velocity_middle_)
         return false;
      if (this.current_velocity_ring_ != otherMyClass.current_velocity_ring_)
         return false;
      if (this.current_velocity_pinky_ != otherMyClass.current_velocity_pinky_)
         return false;
      if (this.current_velocity_thumb_rotator_ != otherMyClass.current_velocity_thumb_rotator_)
         return false;
      if (this.current_velocity_thumb_flexor_ != otherMyClass.current_velocity_thumb_flexor_)
         return false;
      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandStatusMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);
      builder.append(", ");
      builder.append("current_position_index=");
      builder.append(this.current_position_index_);
      builder.append(", ");
      builder.append("current_position_middle=");
      builder.append(this.current_position_middle_);
      builder.append(", ");
      builder.append("current_position_ring=");
      builder.append(this.current_position_ring_);
      builder.append(", ");
      builder.append("current_position_pinky=");
      builder.append(this.current_position_pinky_);
      builder.append(", ");
      builder.append("current_position_thumb_rotator=");
      builder.append(this.current_position_thumb_rotator_);
      builder.append(", ");
      builder.append("current_position_thumb_flexor=");
      builder.append(this.current_position_thumb_flexor_);
      builder.append(", ");
      builder.append("current_velocity_index=");
      builder.append(this.current_velocity_index_);
      builder.append(", ");
      builder.append("current_velocity_middle=");
      builder.append(this.current_velocity_middle_);
      builder.append(", ");
      builder.append("current_velocity_ring=");
      builder.append(this.current_velocity_ring_);
      builder.append(", ");
      builder.append("current_velocity_pinky=");
      builder.append(this.current_velocity_pinky_);
      builder.append(", ");
      builder.append("current_velocity_thumb_rotator=");
      builder.append(this.current_velocity_thumb_rotator_);
      builder.append(", ");
      builder.append("current_velocity_thumb_flexor=");
      builder.append(this.current_velocity_thumb_flexor_);
      builder.append("}");
      return builder.toString();
   }
}
