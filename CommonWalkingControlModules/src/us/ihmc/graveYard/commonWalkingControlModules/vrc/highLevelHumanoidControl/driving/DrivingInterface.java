package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving;

import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

/**
 *
 *
 * throw an exeption if a new command is sent before the previous is finished.
 *
 * @author twan
 *         Date: 5/28/13
 */

public interface DrivingInterface
{
   /**
    * Turn the steering wheel to the desired angle
    * @param angle angle in radians, counterclockwise when facing steering wheel, zero is straight ahead
    */
   public abstract void turnSteeringWheel(double angle);

   /**
    * Press the gas pedal
    * @param distance how far down to press the pedal
    */
   public abstract void pressGasPedal(double distance);

   /**
    * Press the brake pedal
    * @param distance how far down to press the pedal
    */
   public abstract void pressBrakePedal(double distance);

   /**
    * Engage or disengage the hand brake.
    * Will not do anything if already in the right position, except when overrideChecks == true
    * @param engaged
    */
   public abstract void setHandBrake(boolean engaged, boolean overrideChecks);

   /**
    * Change to the desired gear.
    * Will not do anything if already in the right gear, except when overrideChecks == true
    * @param gearName the gear to switch to
    * @param overrideChecks whether or not to override checks
    */
   public abstract void setGear(GearName gearName, boolean overrideChecks);

   /**
    * VRC hack for DO_NOTHING command
    */
   public abstract HumanoidGlobalDataProducer getStatusProducer();
   
   
   public enum GearName
   {
      FORWARD, NEUTRAL, REVERSE
   }


   public abstract void reinitialize();
}
