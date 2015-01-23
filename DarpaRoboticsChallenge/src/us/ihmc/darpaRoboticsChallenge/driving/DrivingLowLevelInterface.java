package us.ihmc.darpaRoboticsChallenge.driving;

import georegression.struct.se.Se3_F64;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;

/**
 * Interface for sending commands to the polaris vehicle in the Darpa Robotics Challenge.  Designed to work in cheat mode and as a high level action
 * to robot.
 */
public interface DrivingLowLevelInterface
{
   public enum Gear {FORWARD, REVERSE}

   public void getInCar();

   /**
    * Initializes everything and blocks until finished.
    *
    *
    * @param address Address the server is running on.  null will use default value
    * @param listenerStereo
    * @param listenerVehiclePose
    */
   public void initialize(String address, DRCStereoListener listenerStereo, PoseListener listenerVehiclePose);

   public void setDirection(Gear gear, boolean block);

   public void setHandBrakeOn(boolean isBrakeOn, boolean block);

   public void setBrakePedal(double value, boolean block);    // 0.0 (none) to 1.0 (full)

   public void setGasPedal(double value, boolean block);    // 0.0 (none) to 1.0 (full)

   public void setSteeringWheelAngle(double angleInRadians, boolean block);    // -pi to pi

   /**
    * If possible, it will kill any action which blocks and return immediately.
    */
   public void interupt();

   /**
    * Waits until the specified time in simulation-time has elapsed
    * @param seconds Number of seconds to wait
    */
   public void drivingWait(double seconds);

   public static interface PoseListener
   {
      void handleVehiclePose(Se3_F64 pose);
   }

}
