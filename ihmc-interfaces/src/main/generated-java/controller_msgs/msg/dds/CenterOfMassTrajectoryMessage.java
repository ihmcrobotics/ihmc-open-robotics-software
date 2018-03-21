package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * Request the whole-body controller to track a desired trajectory for the center of mass.
 */
public class CenterOfMassTrajectoryMessage implements Settable<CenterOfMassTrajectoryMessage>, EpsilonComparable<CenterOfMassTrajectoryMessage>
{
   /**
    * List of center of mass trajectory waypoints. Each waypoint contains the center of mass position and velocity at a given time.
    */
   private controller_msgs.msg.dds.EuclideanTrajectoryMessage euclidean_trajectory_;

   public CenterOfMassTrajectoryMessage()
   {
      euclidean_trajectory_ = new controller_msgs.msg.dds.EuclideanTrajectoryMessage();
   }

   public CenterOfMassTrajectoryMessage(CenterOfMassTrajectoryMessage other)
   {
      set(other);
   }

   public void set(CenterOfMassTrajectoryMessage other)
   {
      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.staticCopy(other.euclidean_trajectory_, euclidean_trajectory_);
   }

   /**
    * List of center of mass trajectory waypoints. Each waypoint contains the center of mass position and velocity at a given time.
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

      if (!this.euclidean_trajectory_.equals(otherMyClass.euclidean_trajectory_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CenterOfMassTrajectoryMessage {");
      builder.append("euclidean_trajectory=");
      builder.append(this.euclidean_trajectory_);

      builder.append("}");
      return builder.toString();
   }
}