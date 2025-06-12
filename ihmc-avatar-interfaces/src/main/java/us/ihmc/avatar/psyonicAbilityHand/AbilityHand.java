package us.ihmc.avatar.psyonicAbilityHand;

public interface AbilityHand
{
   /**
    * Updates the state of this object.
    * Involves writing the command values to the hand
    * and reading the status from the hand.
    */
   void update();

   // Setters for command
   void setIndexFingerPositionCommand(float degrees);
   void setMiddleFingerPositionCommand(float degrees);
   void setRingFingerPositionCommand(float degrees);
   void setPinkyFingerPositionCommand(float degrees);
   void setThumbFlexorPositionCommand(float degrees);
   void setThumbRotatorPositionCommand(float degrees);

   // Getters for status
   float getIndexFingerPositionStatus();
   float getMiddleFingerPositionStatus();
   float getRingFingerPositionStatus();
   float getPinkyFingerPositionStatus();
   float getThumbFlexorPositionStatus();
   float getThumbRotatorPositionStatus();
}
