package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;

/**
 * This is a holder for frame related information that is passed through packages that implement
 * {@link FrameBasedMessage}.
 */
@RosMessagePacket(documentation = "This is a holder for frame related information. Valid codes and their associated frames include:\n"
      + "MIDFEET_ZUP_FRAME = -100\n"
      + "PELVIS_ZUP_FRAME = -101\n"
      + "PELVIS_FRAME = -102\n"
      + "CHEST_FRAME = -103\n"
      + "CENTER_OF_MASS_FRAME = -104\n"
      + "LEFT_SOLE_FRAME = -105\n"
      + "RIGHT_SOLE_FRAME = -106", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, isIHMCPacket = false)
public class FrameInformation extends Packet<FrameInformation>
{
   /**
    * The ID of the reference frame that a trajectory is executed in.
    */
   @RosExportedField(documentation = "The ID of the reference frame that a trajectory is executed in.")
   public long trajectoryReferenceFrameId;

   /**
    * The ID of the reference frame that trajectory data in a packet is expressed in. The frame of the
    * trajectory data will be switched to the trajectory frame immediately when the message is received
    * by the controller. If set to the value {@link NameBasedHashCodeTools#DEFAULT_HASHCODE} it will be
    * assumed that this is the same frame as the trajectory frame.
    * </p>
    * It is recommended that this should be the same frame as the {@link #trajectoryReferenceFrameId} to
    * avoid unexpected behavior. Setting this frame to something different then the trajectory execution
    * frame is equivalent to calling {@link FrameGeometryObject#changeFrame(ReferenceFrame)} on all trajectory
    * data right before it is received by the controller.
    * </p>
    * The data frame is only useful if the user is unable to change the frame the data is expressed in
    * to the trajectory frame. However, unexpected behavior might occur if the data frame is moving
    * with respect to the trajectory frame during execution. To highlight this consider the following
    * example:
    * </p>
    * A hand trajectory needs to be executed while the robot walks to a location in world. The hand
    * trajectory might be known in world frame but for safety the trajectory execution frame is set
    * to a frame attached to the robot. If the data is packed in world and the data frame is set to world
    * this will cause the resulting trajectory to be wrong since the transformation to trajectory frame
    * happens at the start of execution rather than every controller tick.
    */
   @RosExportedField(documentation = "The ID of the reference frame that trajectory data in a packet is expressed in. The frame of the\n"
                                   + "trajectory data will be switched to the trajectory frame immediately when the message is received\n"
                                   + "by the controller.")
   public long dataReferenceFrameId = NameBasedHashCodeTools.DEFAULT_HASHCODE;

   public FrameInformation()
   {
      trajectoryReferenceFrameId = ReferenceFrame.getWorldFrame().getNameBasedHashCode();
   }

   public long getTrajectoryReferenceFrameId()
   {
      return trajectoryReferenceFrameId;
   }

   public void setTrajectoryReferenceFrameId(long trajectoryReferenceFrameId)
   {
      this.trajectoryReferenceFrameId = trajectoryReferenceFrameId;
   }

   public long getDataReferenceFrameId()
   {
      return dataReferenceFrameId;
   }

   public void setDataReferenceFrameId(long dataReferenceFrameId)
   {
      this.dataReferenceFrameId = dataReferenceFrameId;
   }

   @Override
   public void set(FrameInformation other)
   {
      this.trajectoryReferenceFrameId = other.getTrajectoryReferenceFrameId();
      this.dataReferenceFrameId = other.getDataReferenceFrameId();
      setPacketInformation(other);
   }

   public void setTrajectoryReferenceFrame(ReferenceFrame trajectoryFrame)
   {
      setTrajectoryReferenceFrameId(trajectoryFrame.getNameBasedHashCode());
   }

   public void setDataReferenceFrame(ReferenceFrame dataFrame)
   {
      setDataReferenceFrameId(dataFrame.getNameBasedHashCode());
   }

   @Override
   public boolean epsilonEquals(FrameInformation other, double epsilon)
   {
      if (trajectoryReferenceFrameId != other.trajectoryReferenceFrameId)
         return false;
      if (dataReferenceFrameId != other.dataReferenceFrameId)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      if (dataReferenceFrameId == NameBasedHashCodeTools.DEFAULT_HASHCODE)
         return "Trajectory Frame: " + trajectoryReferenceFrameId;
      else
         return "Trajectory Frame: " + trajectoryReferenceFrameId + ", DataFrame: " + dataReferenceFrameId;
   }

   public static void checkIfDataFrameIdsMatch(FrameInformation frameInformation, ReferenceFrame referenceFrame)
   {
      long expectedId = getDataFrameIDConsideringDefault(frameInformation);

      if (expectedId != referenceFrame.getNameBasedHashCode() && expectedId != referenceFrame.getAdditionalNameBasedHashCode())
      {
         String msg = "Argument's hashcode " + referenceFrame + " " + referenceFrame.getNameBasedHashCode() + " does not match " + expectedId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public static void checkIfDataFrameIdsMatch(FrameInformation frameInformation, long otherReferenceFrameId)
   {
      long expectedId = getDataFrameIDConsideringDefault(frameInformation);

      if (expectedId != otherReferenceFrameId)
      {
         String msg = "Argument's hashcode " + otherReferenceFrameId + " does not match " + expectedId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public static long getDataFrameIDConsideringDefault(FrameInformation frameInformation)
   {
      long dataId = frameInformation.getDataReferenceFrameId();
      if (dataId == NameBasedHashCodeTools.DEFAULT_HASHCODE)
      {
         dataId = frameInformation.getTrajectoryReferenceFrameId();
      }
      return dataId;
   }
}
