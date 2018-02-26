package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;

public class PrepareForLocomotionMessage extends Packet<PrepareForLocomotionMessage>
{
   public boolean prepareManipulation = true;
   public boolean preparePelvis = true;

   public PrepareForLocomotionMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(PrepareForLocomotionMessage other)
   {
      prepareManipulation = other.prepareManipulation;
      preparePelvis = other.preparePelvis;
      setPacketInformation(other);
   }

   /**
    * Specifies if the arm controller should be switching to chest frame or jointspace only if
    * necessary. This is particularly useful when manipulation was performed with respect to world
    * during standing to prevent "leaving a hand behind" when the robot starts walking.
    *
    * @param prepareManipulation whether the manipulation control should get prepared for walking.
    */
   public void setPrepareManipulation(boolean prepareManipulation)
   {
      this.prepareManipulation = prepareManipulation;
   }

   /**
    * Specifies if the pelvis orientation controller should be initialized before starting to walk.
    * When the controller is initialized, the pelvis will smoothly cancel out the user orientation
    * offset on the first transfer of a walking sequence.
    *
    * @param preparePelvis whether the pelvis orientation control should get prepared for walking.
    */
   public void setPreparePelvis(boolean preparePelvis)
   {
      this.preparePelvis = preparePelvis;
   }

   public boolean isPrepareManipulation()
   {
      return prepareManipulation;
   }

   public boolean isPreparePelvis()
   {
      return preparePelvis;
   }

   @Override
   public boolean epsilonEquals(PrepareForLocomotionMessage other, double epsilon)
   {
      if (prepareManipulation != other.prepareManipulation)
         return false;
      if (preparePelvis != other.preparePelvis)
         return false;

      return true;
   }
}
