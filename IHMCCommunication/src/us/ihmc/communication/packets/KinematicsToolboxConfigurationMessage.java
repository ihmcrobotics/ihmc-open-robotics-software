package us.ihmc.communication.packets;

/**
 * {@link KinematicsToolboxConfigurationMessage} is part of the API of the
 * {@code KinematicsToolboxController}.
 * <p>
 * It contains auxiliary information that allows to further customized the behavior of the solver.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class KinematicsToolboxConfigurationMessage extends TrackablePacket<KinematicsToolboxConfigurationMessage>
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
   public boolean holdSupporFootPositions = true;

   public KinematicsToolboxConfigurationMessage()
   {
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
    * @param holdSupporFootPositions whether the support foot/feet should be held in place.
    */
   public void setHoldSupporFootPositions(boolean holdSupporFootPositions)
   {
      this.holdSupporFootPositions = holdSupporFootPositions;
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean holdSupporFootPositions()
   {
      return holdSupporFootPositions;
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
   public boolean epsilonEquals(KinematicsToolboxConfigurationMessage other, double epsilon)
   {
      if (holdCurrentCenterOfMassXYPosition != other.holdCurrentCenterOfMassXYPosition)
         return false;
      if (holdSupporFootPositions != other.holdSupporFootPositions)
         return false;
      return true;
   }
}
