package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation = "This mesage commands the controller to move the pelvis to a new height in the trajectory frame while going through the specified trajectory points."
      + " Sending this command will not affect the pelvis horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/pelvis_height_trajectory")
public class PelvisHeightTrajectoryMessage extends Packet<PelvisHeightTrajectoryMessage>
{
   @RosExportedField(documentation = "Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.")
   public boolean enableUserPelvisControl = false;

   @RosExportedField(documentation = " If enableUserPelvisControl is true then enableUserPelvisControlDuringWalking"
         + " will keep the height manager in user mode while walking. If this is false the height manager will switch to controller mode when walking.")
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
      euclideanTrajectory.selectionMatrix.xSelected = false;
      euclideanTrajectory.selectionMatrix.ySelected = false;
      euclideanTrajectory.selectionMatrix.zSelected = true;
   }

   /**
    * Clone contructor.
    * 
    * @param other message to clone.
    */
   public PelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage other)
   {
      euclideanTrajectory = new EuclideanTrajectoryMessage(other.euclideanTrajectory);
      setUniqueId(other.getUniqueId());
      setDestination(other.getDestination());

      enableUserPelvisControl = other.enableUserPelvisControl;
      enableUserPelvisControlDuringWalking = other.getEnableUserPelvisControlDuringWalking();
   }

   @Override
   public void set(PelvisHeightTrajectoryMessage other)
   {
      enableUserPelvisControl = other.enableUserPelvisControl;
      enableUserPelvisControlDuringWalking = other.enableUserPelvisControlDuringWalking;
      euclideanTrajectory = new EuclideanTrajectoryMessage();
      euclideanTrajectory.set(other.euclideanTrajectory);
   }

   /**
    * Returns whether or not user mode is enabled. If enabled the controller will execute the
    * trajectory in user mode. User mode will try to achieve the desireds regardless of the leg
    * kinematics
    * 
    * @return whether or not user mode is enabled.
    */
   public boolean getEnableUserPelvisControl()
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
   public boolean getEnableUserPelvisControlDuringWalking()
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
         return "Pelvis height trajectory: number of trajectory points = " + euclideanTrajectory.taskspaceTrajectoryPoints.size();
      else
         return "Pelvis height trajectory: no trajectory points   .";
   }
}
