package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. Request the whole-body controller to
 * track a desired trajectory for the center of mass.
 */
public class CenterOfMassTrajectoryMessage extends Packet<CenterOfMassTrajectoryMessage>
      implements Settable<CenterOfMassTrajectoryMessage>, EpsilonComparable<CenterOfMassTrajectoryMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * List of center of mass trajectory waypoints. Each waypoint contains the center of mass
    * position and velocity at a given time.
    */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage euclidean_trajectory_;

   public CenterOfMassTrajectoryMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      euclidean_trajectory_ = new controller_msgs.msg.dds.EuclideanTrajectoryMessage();
   }

   public CenterOfMassTrajectoryMessage(CenterOfMassTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(CenterOfMassTrajectoryMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.staticCopy(other.euclidean_trajectory_, euclidean_trajectory_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * List of center of mass trajectory waypoints. Each waypoint contains the center of mass
    * position and velocity at a given time.
    */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage getEuclideanTrajectory()
   {
      return euclidean_trajectory_;
   }

   @Override
   public boolean epsilonEquals(CenterOfMassTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.euclidean_trajectory_.epsilonEquals(other.euclidean_trajectory_, epsilon))
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
      if (!(other instanceof CenterOfMassTrajectoryMessage))
         return false;

      CenterOfMassTrajectoryMessage otherMyClass = (CenterOfMassTrajectoryMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.euclidean_trajectory_.equals(otherMyClass.euclidean_trajectory_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CenterOfMassTrajectoryMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("euclidean_trajectory=");
      builder.append(this.euclidean_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
