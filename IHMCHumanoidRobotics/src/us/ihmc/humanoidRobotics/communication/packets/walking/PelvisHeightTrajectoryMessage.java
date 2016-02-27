package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.packets.Abstract1DTrajectoryMessage;

@ClassDocumentation("This mesage commands the controller to move the pelvis to a new height in world while going through the specified trajectory points."
      + " Sending this command will not affect the pelvis horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead."
      + " A third order polynomial is used to interpolate between trajectory points."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class PelvisHeightTrajectoryMessage extends Abstract1DTrajectoryMessage<PelvisHeightTrajectoryMessage> implements VisualizablePacket
{
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisHeightTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone contructor.
    * @param pelvisHeightTrajectoryMessage message to clone.
    */
   public PelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      super(pelvisHeightTrajectoryMessage);
      setUniqueId(pelvisHeightTrajectoryMessage.getUniqueId());
      setDestination(pelvisHeightTrajectoryMessage.getDestination());
   }

   /**
    * Use this constructor to go straight to the given end point.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in world frame.
    */
   public PelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      super(trajectoryTime, desiredHeight);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, double, double)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public PelvisHeightTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public boolean epsilonEquals(PelvisHeightTrajectoryMessage other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
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
