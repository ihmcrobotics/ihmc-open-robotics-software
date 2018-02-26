package us.ihmc.communication.packets;

public class HumanoidKinematicsToolboxConfigurationMessage extends Packet<HumanoidKinematicsToolboxConfigurationMessage>
{
   /**
    * When set to {@code true}, the solver will hold the current x and y coordinates of the center
    * of mass. By 'current', it means that the solver will use the robot configuration data
    * broadcasted by the controller to obtain the center of mass position.
    */
   public boolean holdCurrentCenterOfMassXYPosition = true;
   /**
    * When set to {@code true}, the solver will hold the pose of the active support foot/feet.
    */
   public boolean holdSupportFootPositions = true;

   public HumanoidKinematicsToolboxConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(HumanoidKinematicsToolboxConfigurationMessage other)
   {
      holdCurrentCenterOfMassXYPosition = other.holdCurrentCenterOfMassXYPosition;
      holdSupportFootPositions = other.holdSupportFootPositions;
      setPacketInformation(other);
   }

   /**
    * Specifies whether the {@code KinematicsToolboxController} should hold the initial x and y
    * coordinates of the robot's center of mass.
    * 
    * @param holdCurrentCenterOfMassXYPosition whether x and y coordinates of the robot's center of
    *           mass should be held or not.
    */
   public void setHoldCurrentCenterOfMassXYPosition(boolean holdCurrentCenterOfMassXYPosition)
   {
      this.holdCurrentCenterOfMassXYPosition = holdCurrentCenterOfMassXYPosition;
   }

   /**
    * Specifies whether the {@code KinematicsToolboxController} should the initial pose of the
    * current support foot/feet.
    * 
    * @param holdSupportFootPositions whether the support foot/feet should be held in place.
    */
   public void setHoldSupportFootPositions(boolean holdSupportFootPositions)
   {
      this.holdSupportFootPositions = holdSupportFootPositions;
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean holdSupportFootPositions()
   {
      return holdSupportFootPositions;
   }

   /**
    * Compares each field of this message against the other message and returns {@code true} if they
    * are equal to an {@code epsilon}.
    * <p>
    * Note that this method considers two fields to be equal if they are both {@code null}, and
    * considers two fields to be different if only one is equal to {@code null}.
    * </p>
    * 
    * @return {@code true} if the two messages are equal to an {@code epsilon}, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(HumanoidKinematicsToolboxConfigurationMessage other, double epsilon)
   {
      if (holdCurrentCenterOfMassXYPosition != other.holdCurrentCenterOfMassXYPosition)
         return false;
      if (holdSupportFootPositions != other.holdSupportFootPositions)
         return false;
      return true;
   }

   
}
