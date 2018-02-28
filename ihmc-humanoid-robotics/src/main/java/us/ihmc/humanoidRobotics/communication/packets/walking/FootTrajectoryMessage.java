package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryMessage;

@RosMessagePacket(documentation =
"This message commands the controller first to unload if necessary and then to move in taskspace a foot to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/foot_trajectory")
public class FootTrajectoryMessage extends Packet<FootTrajectoryMessage>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   @RosExportedField(documentation = "Specifies which foot will execute the trajectory.")
   public byte robotSide;
   @RosExportedField(documentation = "The position/orientation trajectory information.")
   public SE3TrajectoryMessage se3Trajectory;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param footTrajectoryMessage message to clone.
    */
   public FootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      se3Trajectory = new SE3TrajectoryMessage(footTrajectoryMessage.se3Trajectory);
      robotSide = footTrajectoryMessage.robotSide;
      setUniqueId(footTrajectoryMessage.getUniqueId());
      setDestination(footTrajectoryMessage.getDestination());
   }

   @Override
   public void set(FootTrajectoryMessage other)
   {
      se3Trajectory = new SE3TrajectoryMessage(other.se3Trajectory);
      robotSide = other.robotSide;
      setPacketInformation(other);
   }

   public void setRobotSide(byte robotSide)
   {
      this.robotSide = robotSide;
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public void setSe3Trajectory(SE3TrajectoryMessage se3Trajectory)
   {
      this.se3Trajectory = se3Trajectory;
   }

   public SE3TrajectoryMessage getSe3Trajectory()
   {
      return se3Trajectory;
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (se3Trajectory != null)
         se3Trajectory.setUniqueId(uniqueId);
   }

   @Override
   public boolean epsilonEquals(FootTrajectoryMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;

      return se3Trajectory.epsilonEquals(other.se3Trajectory, epsilon);
   }

   @Override
   public String toString()
   {
      String ret = "";
      if (se3Trajectory.taskspaceTrajectoryPoints != null)
         ret = "Foot SE3 trajectory: number of SE3 trajectory points = " + se3Trajectory.getNumberOfTrajectoryPoints();
      else
         ret = "Foot SE3 trajectory: no SE3 trajectory points";

      return ret + ", robotSide = " + robotSide + ".";
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootTrajectoryMessage(this);
   }
}
