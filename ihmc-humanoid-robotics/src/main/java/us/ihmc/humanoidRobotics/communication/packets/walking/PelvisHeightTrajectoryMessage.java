package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation = "This mesage commands the controller to move the pelvis to a new height in the trajectory frame while going through the specified trajectory points."
      + " Sending this command will not affect the pelvis horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/pelvis_height_trajectory")
public class PelvisHeightTrajectoryMessage extends Packet<PelvisHeightTrajectoryMessage>
{
   /**
    * Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the
    * leg kinematics
    **/
   public boolean enableUserPelvisControl = false;

   /**
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking}
    * will keep the height manager in user mode while walking. If this is false the height manager
    * will switch to controller mode when walking
    **/
   public boolean enableUserPelvisControlDuringWalking = false;
   @RosExportedField(documentation = "The position trajectory information.")
   public EuclideanTrajectoryMessage euclideanTrajectory;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisHeightTrajectoryMessage()
   {
      euclideanTrajectory = new EuclideanTrajectoryMessage();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      euclideanTrajectory.selectionMatrix.setAxisSelection(false, false, true);
   }

   /**
    * Clone contructor.
    * 
    * @param pelvisHeightTrajectoryMessage message to clone.
    */
   public PelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      euclideanTrajectory = new EuclideanTrajectoryMessage(pelvisHeightTrajectoryMessage.euclideanTrajectory);
      setUniqueId(pelvisHeightTrajectoryMessage.getUniqueId());
      setDestination(pelvisHeightTrajectoryMessage.getDestination());

      enableUserPelvisControl = pelvisHeightTrajectoryMessage.enableUserPelvisControl;
      enableUserPelvisControlDuringWalking = pelvisHeightTrajectoryMessage.isEnableUserPelvisControlDuringWalking();
   }

   /**
    * Use this constructor to go straight to the given end point. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in data frame
    * @param trajectoryReferenceFrame the frame in which the height will be executed
    * @param dataReferenceFrame the frame the desiredHeight is expressed in, the height will be
    *           changed to the trajectory frame on the controller side
    */
   public PelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight, ReferenceFrame trajectoryReferenceFrame, ReferenceFrame dataReferenceFrame)
   {
      euclideanTrajectory = new EuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight),
                                                           trajectoryReferenceFrame.getNameBasedHashCode());
      euclideanTrajectory.frameInformation.setDataReferenceFrame(dataReferenceFrame);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      euclideanTrajectory.selectionMatrix.setAxisSelection(false, false, true);
   }

   /**
    * Use this constructor to go straight to the given end point. The trajectory and data frame are
    * set to world frame Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in world frame.
    */
   public PelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      euclideanTrajectory = new EuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight), ReferenceFrame.getWorldFrame());
      euclideanTrajectory.frameInformation.setDataReferenceFrame(ReferenceFrame.getWorldFrame());
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      euclideanTrajectory.selectionMatrix.setAxisSelection(false, false, true);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint}
    * for each trajectory point afterwards. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public PelvisHeightTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      euclideanTrajectory = new EuclideanTrajectoryMessage(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      euclideanTrajectory.selectionMatrix.setAxisSelection(false, false, true);
   }

   /**
    * Returns whether or not user mode is enabled. If enabled the controller will execute the
    * trajectory in user mode. User mode will try to achieve the desireds regardless of the leg
    * kinematics
    * 
    * @return whether or not user mode is enabled.
    */
   public boolean isEnableUserPelvisControl()
   {
      return enableUserPelvisControl;
   }

   /**
    * If enabled the controller will execute the trajectory in user mode. User mode will try to
    * achieve the desireds regardless of the leg kinematics
    */
   public void setEnableUserPelvisControl(boolean enableUserPelvisControl)
   {
      this.enableUserPelvisControl = enableUserPelvisControl;
   }

   /**
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking}
    * will keep the height manager in user mode while walking. If this is false the height manager
    * will switch to controller mode when walking
    * 
    * @return whether or not user mode is enabled while walking
    **/
   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   /**
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking}
    * will keep the height manager in user mode while walking. If this is false the height manager
    * will switch to controller mode when walking
    * 
    * @param enableUserPelvisControlDuringWalking sets whether or not user mode is enabled while
    *           walking
    **/
   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (euclideanTrajectory != null)
         euclideanTrajectory.setUniqueId(uniqueId);
   }

   public EuclideanTrajectoryMessage getEuclideanTrajectory()
   {
      return euclideanTrajectory;
   }

   /**
    * Returns whether this message is valid
    * 
    * @return returns null if the message is valid, returns a string describing why the message is
    *         invalid if it is invalid
    */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validatePelvisHeightTrajectoryMessage(this);
   }

   /**
    * Compares two objects are equal to within some epsilon
    */
   @Override
   public boolean epsilonEquals(PelvisHeightTrajectoryMessage other, double epsilon)
   {
      if (enableUserPelvisControl != other.enableUserPelvisControl)
      {
         return false;
      }
      if (enableUserPelvisControlDuringWalking != other.enableUserPelvisControlDuringWalking)
      {
         return false;
      }
      return euclideanTrajectory.epsilonEquals(other.euclideanTrajectory, epsilon);
   }

   @Override
   public String toString()
   {
      if (euclideanTrajectory.taskspaceTrajectoryPoints != null)
         return "Pelvis height trajectory: number of trajectory points = " + euclideanTrajectory.getNumberOfTrajectoryPoints();
      else
         return "Pelvis height trajectory: no trajectory points   .";
   }
}
