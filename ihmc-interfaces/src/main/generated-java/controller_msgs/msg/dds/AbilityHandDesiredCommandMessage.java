package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class AbilityHandDesiredCommandMessage extends Packet<AbilityHandDesiredCommandMessage>
      implements Settable<AbilityHandDesiredCommandMessage>, EpsilonComparable<AbilityHandDesiredCommandMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * Specifies the side of the robot of the hand being referred to
    */
   public byte robot_side_ = (byte) 255;

   public double[] gripper_desired_position_ = new double[3];

   public AbilityHandDesiredCommandMessage()
   {
   }

   public AbilityHandDesiredCommandMessage(AbilityHandDesiredCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandDesiredCommandMessage other)
   {
      robot_side_ = other.robot_side_;
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
   public void setDesiredPosition(double[] gripper_desired_position)
   {
      gripper_desired_position_ = gripper_desired_position;
   }
   public double[] getDesiredPosition()
   {
      return gripper_desired_position_;
   }

   //Remove when class is not an error
   @Override
   public boolean epsilonEquals(AbilityHandDesiredCommandMessage other, double epsilon)
   {
      return false;
   }
}
