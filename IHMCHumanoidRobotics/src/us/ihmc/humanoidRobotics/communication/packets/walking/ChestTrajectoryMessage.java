package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3TrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;

@RosMessagePacket(documentation =
      "This message commands the controller to move in taskspace the chest to the desired orientation while going through the specified trajectory points."
      + " A hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a simple trajectory to reach a desired chest orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/chest_trajectory")
public class ChestTrajectoryMessage extends AbstractSO3TrajectoryMessage<ChestTrajectoryMessage> implements VisualizablePacket
{
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ChestTrajectoryMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public ChestTrajectoryMessage(Random random)
   {
      super(random);
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public ChestTrajectoryMessage(AbstractSO3TrajectoryMessage<?> chestTrajectoryMessage)
   {
      super(chestTrajectoryMessage);
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed in World.
    */
   public ChestTrajectoryMessage(double trajectoryTime, Quaternion desiredOrientation, long expressedInReferenceFrameId, long trajectoryReferenceFrameID)
   {
      super(trajectoryTime, desiredOrientation, expressedInReferenceFrameId, trajectoryReferenceFrameID);
   }
   
   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed the supplied frame.
    */
   public ChestTrajectoryMessage(double trajectoryTime, Quaternion desiredOrientation, ReferenceFrame expressedInReferenceFrame, ReferenceFrame trajectoryFrame)
   {
      super(trajectoryTime, desiredOrientation, expressedInReferenceFrame, trajectoryFrame);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * By default this constructor sets the trajectory frame to pelvis z up and the data frame to world
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Quaternion, Vector3D)} for each trajectory point afterwards.
    * Sets the frame to control in to world
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public ChestTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      super.setTrajectoryReferenceFrameId(CommonReferenceFrameIds.PELVIS_ZUP_FRAME.getHashId());
      super.setDataReferenceFrameId(ReferenceFrame.getWorldFrame());
   }

}
