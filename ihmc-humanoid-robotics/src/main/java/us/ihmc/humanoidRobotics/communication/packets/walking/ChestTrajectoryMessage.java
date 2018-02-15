package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;

@RosMessagePacket(documentation = "This message commands the controller to move in taskspace the chest to the desired orientation while going through the specified trajectory points."
      + " A hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a simple trajectory to reach a desired chest orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/chest_trajectory")
public class ChestTrajectoryMessage extends Packet<ChestTrajectoryMessage>
{
   @RosExportedField(documentation = "The orientation trajectory information.")
   public SO3TrajectoryMessage so3Trajectory;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ChestTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public ChestTrajectoryMessage(SO3TrajectoryMessage so3Trajectory)
   {
      this.so3Trajectory = new SO3TrajectoryMessage(so3Trajectory);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * 
    * @param message to clone.
    */
   public ChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      so3Trajectory = new SO3TrajectoryMessage(chestTrajectoryMessage.so3Trajectory);
      setUniqueId(chestTrajectoryMessage.getUniqueId());
      setDestination(chestTrajectoryMessage.getDestination());
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed in World.
    */
   public ChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, long trajectoryReferenceFrameID)
   {
      so3Trajectory = new SO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryReferenceFrameID);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed the supplied frame.
    */
   public ChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryFrame)
   {
      so3Trajectory = new SO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public ChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly quaternion, ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame)
   {
      this(trajectoryTime, quaternion, trajectoryFrame);
      so3Trajectory.getFrameInformation().setDataReferenceFrame(dataFrame);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. By default this
    * constructor sets the trajectory frame to pelvis z up and the data frame to world This
    * constructor only allocates memory for the trajectory points, you need to call
    * {@link #setTrajectoryPoint(int, double, Quaternion, Vector3D)} for each trajectory point
    * afterwards. Sets the frame to control in to world
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public ChestTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      so3Trajectory = new SO3TrajectoryMessage(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(ChestTrajectoryMessage other)
   {
      so3Trajectory = new SO3TrajectoryMessage();
      so3Trajectory.set(other.so3Trajectory);
      setPacketInformation(other);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (so3Trajectory != null)
         so3Trajectory.setUniqueId(uniqueId);
   }

   public SO3TrajectoryMessage getSO3Trajectory()
   {
      return so3Trajectory;
   }

   @Override
   public boolean epsilonEquals(ChestTrajectoryMessage other, double epsilon)
   {
      if (!so3Trajectory.epsilonEquals(other.so3Trajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateChestTrajectoryMessage(this);
   }
}
