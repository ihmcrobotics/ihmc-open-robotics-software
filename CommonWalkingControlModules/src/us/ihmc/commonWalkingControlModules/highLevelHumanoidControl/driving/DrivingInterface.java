package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving;

/**
 * @author twan
 *         Date: 5/28/13
 */

// TODO: add callbacks
public interface DrivingInterface
{
   public abstract void turnSteeringWheel(double angle);

   public abstract void pressGasPedal(double angle);

   public abstract void pressBrakePedal(double angle);

   public abstract void setParkingBrake(boolean engaged);

   public abstract void setGear(GearName gearName);

   enum GearName
   {
      FORWARD, NEUTRAL, REVERSE
   }
}
