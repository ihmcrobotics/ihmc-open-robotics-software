package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * This message commands the controller to move the hand in both taskspace and jointspace
 * to the desired orientation and joint angles while going through the specified trajectory points.
 */
public class HandHybridJointspaceTaskspaceTrajectoryMessage extends Packet<HandHybridJointspaceTaskspaceTrajectoryMessage>
      implements Settable<HandHybridJointspaceTaskspaceTrajectoryMessage>, EpsilonComparable<HandHybridJointspaceTaskspaceTrajectoryMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * Specifies the side of the robot that will execute the trajectory.
    */
   public byte robot_side_ = (byte) 255;
   /**
    * The taskspace trajectory information.
    */
   public controller_msgs.msg.dds.SO3TrajectoryMessage taskspace_trajectory_message_;
   /**
    * The jointspace trajectory information.
    * The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
    */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage jointspace_trajectory_message_;

   public HandHybridJointspaceTaskspaceTrajectoryMessage()
   {

      taskspace_trajectory_message_ = new controller_msgs.msg.dds.SO3TrajectoryMessage();
      jointspace_trajectory_message_ = new controller_msgs.msg.dds.JointspaceTrajectoryMessage();
   }

   public HandHybridJointspaceTaskspaceTrajectoryMessage(HandHybridJointspaceTaskspaceTrajectoryMessage other)
   {
      set(other);
   }

   public void set(HandHybridJointspaceTaskspaceTrajectoryMessage other)
   {
      robot_side_ = other.robot_side_;

      controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.staticCopy(other.taskspace_trajectory_message_, taskspace_trajectory_message_);
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.staticCopy(other.jointspace_trajectory_message_, jointspace_trajectory_message_);
   }

   /**
    * Specifies the side of the robot that will execute the trajectory.
    */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
    * Specifies the side of the robot that will execute the trajectory.
    */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   /**
    * The taskspace trajectory information.
    */
   public controller_msgs.msg.dds.SO3TrajectoryMessage getTaskspaceTrajectoryMessage()
   {
      return taskspace_trajectory_message_;
   }

   /**
    * The jointspace trajectory information.
    * The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
    */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage getJointspaceTrajectoryMessage()
   {
      return jointspace_trajectory_message_;
   }

   @Override
   public boolean epsilonEquals(HandHybridJointspaceTaskspaceTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      if (!this.taskspace_trajectory_message_.epsilonEquals(other.taskspace_trajectory_message_, epsilon))
         return false;

      if (!this.jointspace_trajectory_message_.epsilonEquals(other.jointspace_trajectory_message_, epsilon))
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
      if (!(other instanceof HandHybridJointspaceTaskspaceTrajectoryMessage))
         return false;

      HandHybridJointspaceTaskspaceTrajectoryMessage otherMyClass = (HandHybridJointspaceTaskspaceTrajectoryMessage) other;

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      if (!this.taskspace_trajectory_message_.equals(otherMyClass.taskspace_trajectory_message_))
         return false;

      if (!this.jointspace_trajectory_message_.equals(otherMyClass.jointspace_trajectory_message_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandHybridJointspaceTaskspaceTrajectoryMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);

      builder.append(", ");
      builder.append("taskspace_trajectory_message=");
      builder.append(this.taskspace_trajectory_message_);

      builder.append(", ");
      builder.append("jointspace_trajectory_message=");
      builder.append(this.jointspace_trajectory_message_);

      builder.append("}");
      return builder.toString();
   }
}