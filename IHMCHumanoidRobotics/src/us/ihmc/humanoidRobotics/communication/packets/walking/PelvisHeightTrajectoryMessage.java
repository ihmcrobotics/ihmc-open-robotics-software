package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;

@ClassDocumentation("This mesage commands the controller to move the pelvis to a new height in world while going through the specified trajectory points."
      + " Sending this command will not affect the pelvis horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead."
      + " A third order polynomial is used to interpolate between trajectory points."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class PelvisHeightTrajectoryMessage extends IHMCRosApiMessage<PelvisHeightTrajectoryMessage> implements VisualizablePacket
{
   @FieldDocumentation("List of trajectory points to go through while executing the trajectory.")
   public TrajectoryPoint1DMessage[] trajectoryPoints;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisHeightTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone contructor.
    * @param pelvisHeightTrajectoryMessage message to clone.
    */
   public PelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      setUniqueId(pelvisHeightTrajectoryMessage.getUniqueId());
      setDestination(pelvisHeightTrajectoryMessage.getDestination());
      trajectoryPoints = new TrajectoryPoint1DMessage[pelvisHeightTrajectoryMessage.getNumberOfTrajectoryPoints()];
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         trajectoryPoints[i] = new TrajectoryPoint1DMessage(pelvisHeightTrajectoryMessage.trajectoryPoints[i]);
   }

   /**
    * Use this constructor to go straight to the given end point.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in world frame.
    */
   public PelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      trajectoryPoints = new TrajectoryPoint1DMessage[] {new TrajectoryPoint1DMessage(trajectoryTime, desiredHeight, 0.0)};
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, double, double)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public PelvisHeightTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      trajectoryPoints = new TrajectoryPoint1DMessage[numberOfTrajectoryPoints];
   }

   /**
    * Create a trajectory point.
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
    * @param height define the desired height position to be reached at this trajectory point. It is expressed in world frame.
    * @param heightVelocity define the desired height velocity to be reached at this trajectory point. It is expressed in world frame.
    */
   public void setTrajectoryPoint(int trajectoryPointIndex, double time, double height, double heightVelocity)
   {
      rangeCheck(trajectoryPointIndex);
      trajectoryPoints[trajectoryPointIndex] = new TrajectoryPoint1DMessage(time, height, heightVelocity);
   }

   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.length;
   }

   public TrajectoryPoint1DMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints[trajectoryPointIndex];
   }

   public TrajectoryPoint1DMessage[] getTrajectoryPoints()
   {
      return trajectoryPoints;
   }

   private void rangeCheck(int trajectoryPointIndex)
   {
      if (trajectoryPointIndex >= getNumberOfTrajectoryPoints() || trajectoryPointIndex < 0)
         throw new IndexOutOfBoundsException("Trajectory point index: " + trajectoryPointIndex + ", number of trajectory points: " + getNumberOfTrajectoryPoints());
   }

   @Override
   public boolean epsilonEquals(PelvisHeightTrajectoryMessage other, double epsilon)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         return false;
      
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         if (!trajectoryPoints[i].epsilonEquals(other.trajectoryPoints[i], epsilon))
            return false;
      }

      return true;
   }

   @Override
   public String toString()
   {
      if (trajectoryPoints != null)
         return "Pelvis height 1D trajectory: number of 1D trajectory points = " + getNumberOfTrajectoryPoints();
      else
         return "Pelvis height 1D trajectory: no 1D trajectory point.";
   }
}
