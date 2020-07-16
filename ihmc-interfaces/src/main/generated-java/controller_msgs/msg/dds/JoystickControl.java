package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class JoystickControl extends Packet<JoystickControl> implements Settable<JoystickControl>, EpsilonComparable<JoystickControl>
{
   /**
            * This message is sent to command the robot to move as if controlled by a joystick.
            * The command is not an absolute positional goal, but rather a direction of travel and speed
            * represented as a set of 3 movement impulses:
            * - forwards/backwards
            * - left/right
            * - clockwise/counter-clockwise rotation
            * 
            * Each impulse value is a floating point number, v, with -1 <= v <= 1, where
            * v<0 indicates backwards/left/counter-clockwise movement at (-v)*max speed,
            * v=0 indicates no change,
            * v>0 indicates forwards/right/clockwise at v*max speed.
            */
   public double forward_;
   public double right_;
   public double turn_;

   public JoystickControl()
   {
   }

   public JoystickControl(JoystickControl other)
   {
      this();
      set(other);
   }

   public void set(JoystickControl other)
   {
      forward_ = other.forward_;

      right_ = other.right_;

      turn_ = other.turn_;

   }

   /**
            * This message is sent to command the robot to move as if controlled by a joystick.
            * The command is not an absolute positional goal, but rather a direction of travel and speed
            * represented as a set of 3 movement impulses:
            * - forwards/backwards
            * - left/right
            * - clockwise/counter-clockwise rotation
            * 
            * Each impulse value is a floating point number, v, with -1 <= v <= 1, where
            * v<0 indicates backwards/left/counter-clockwise movement at (-v)*max speed,
            * v=0 indicates no change,
            * v>0 indicates forwards/right/clockwise at v*max speed.
            */
   public void setForward(double forward)
   {
      forward_ = forward;
   }
   /**
            * This message is sent to command the robot to move as if controlled by a joystick.
            * The command is not an absolute positional goal, but rather a direction of travel and speed
            * represented as a set of 3 movement impulses:
            * - forwards/backwards
            * - left/right
            * - clockwise/counter-clockwise rotation
            * 
            * Each impulse value is a floating point number, v, with -1 <= v <= 1, where
            * v<0 indicates backwards/left/counter-clockwise movement at (-v)*max speed,
            * v=0 indicates no change,
            * v>0 indicates forwards/right/clockwise at v*max speed.
            */
   public double getForward()
   {
      return forward_;
   }

   public void setRight(double right)
   {
      right_ = right;
   }
   public double getRight()
   {
      return right_;
   }

   public void setTurn(double turn)
   {
      turn_ = turn;
   }
   public double getTurn()
   {
      return turn_;
   }


   public static Supplier<JoystickControlPubSubType> getPubSubType()
   {
      return JoystickControlPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JoystickControlPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JoystickControl other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.forward_, other.forward_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_, other.right_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.turn_, other.turn_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JoystickControl)) return false;

      JoystickControl otherMyClass = (JoystickControl) other;

      if(this.forward_ != otherMyClass.forward_) return false;

      if(this.right_ != otherMyClass.right_) return false;

      if(this.turn_ != otherMyClass.turn_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JoystickControl {");
      builder.append("forward=");
      builder.append(this.forward_);      builder.append(", ");
      builder.append("right=");
      builder.append(this.right_);      builder.append(", ");
      builder.append("turn=");
      builder.append(this.turn_);
      builder.append("}");
      return builder.toString();
   }
}
