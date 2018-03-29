package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message commands the controller
 * to move in taskspace a foot to the desired pose (position & orientation) while going through the
 * specified trajectory points. A third order polynomial function is used to interpolate positions
 * and a Hermite based curve (third order) is used to interpolate the orientations. To execute a
 * single straight line trajectory to reach a desired foot pose, set only one trajectory point with
 * zero velocity and its time to be equal to the desired trajectory time.
 */
public class FootTrajectoryMessage extends Packet<FootTrajectoryMessage> implements Settable<FootTrajectoryMessage>, EpsilonComparable<FootTrajectoryMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * Specifies which foot will execute the trajectory.
    */
   public byte robot_side_ = (byte) 255;
   /**
    * The position/orientation trajectory information.
    */
   public controller_msgs.msg.dds.SE3TrajectoryMessage se3_trajectory_;

   public FootTrajectoryMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      se3_trajectory_ = new controller_msgs.msg.dds.SE3TrajectoryMessage();
   }

   public FootTrajectoryMessage(FootTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(FootTrajectoryMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      robot_side_ = other.robot_side_;

      controller_msgs.msg.dds.SE3TrajectoryMessagePubSubType.staticCopy(other.se3_trajectory_, se3_trajectory_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * Specifies which foot will execute the trajectory.
    */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   /**
    * Specifies which foot will execute the trajectory.
    */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
    * The position/orientation trajectory information.
    */
   public controller_msgs.msg.dds.SE3TrajectoryMessage getSe3Trajectory()
   {
      return se3_trajectory_;
   }

   @Override
   public boolean epsilonEquals(FootTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      if (!this.se3_trajectory_.epsilonEquals(other.se3_trajectory_, epsilon))
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
      if (!(other instanceof FootTrajectoryMessage))
         return false;

      FootTrajectoryMessage otherMyClass = (FootTrajectoryMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      if (!this.se3_trajectory_.equals(otherMyClass.se3_trajectory_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootTrajectoryMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);
      builder.append(", ");
      builder.append("se3_trajectory=");
      builder.append(this.se3_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
