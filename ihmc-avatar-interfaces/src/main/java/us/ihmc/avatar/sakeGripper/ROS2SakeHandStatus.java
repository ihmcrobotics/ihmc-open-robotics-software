package us.ihmc.avatar.sakeGripper;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;

public class ROS2SakeHandStatus
{
   private volatile boolean isCalibrated = false;
   private volatile boolean needsReset;

   private volatile boolean isTorqueOn;
   private volatile double commandedHandOpenAngle = Double.NaN;
   private volatile double commandedFingertipGripForceLimit = Double.NaN;
   private volatile double currentHandOpenAngle = Double.NaN;
   private volatile double currentFingertipGripForce = Double.NaN;
   private volatile double currentTemperature = Double.NaN;
   private volatile double currentVelocity = Double.NaN;
   private volatile int errorCodes = 0;
   private volatile int handRealtimeTick = 0;

   public ROS2SakeHandStatus(ROS2NodeInterface ros2Node, String robotName, RobotSide handSide)
   {
      ROS2Tools.createVolatileCallbackSubscription(ros2Node, ROS2Tools.getEtherSnackHandStatusTopic(robotName, handSide), sakeHandStatusMessage ->
      {
         isCalibrated = sakeHandStatusMessage.getIsCalibrated();
         needsReset = sakeHandStatusMessage.getNeedsReset();
         isTorqueOn = sakeHandStatusMessage.getTorqueOn();
         currentTemperature = sakeHandStatusMessage.getMeasuredTemperature();
         currentHandOpenAngle = SakeHandParameters.denormalizeHandOpenAngle(sakeHandStatusMessage.getMeasuredPosition());
         commandedHandOpenAngle = SakeHandParameters.denormalizeHandOpenAngle(sakeHandStatusMessage.getDesiredPosition());
         currentFingertipGripForce = SakeHandParameters.denormalizeFingertipGripForceLimit(sakeHandStatusMessage.getMeasuredTorque());
         commandedFingertipGripForceLimit = SakeHandParameters.denormalizeFingertipGripForceLimit(sakeHandStatusMessage.getTorqueLimit());
         currentVelocity = sakeHandStatusMessage.getMeasuredVelocity();
         errorCodes = sakeHandStatusMessage.getErrorCodes();
         handRealtimeTick = sakeHandStatusMessage.getRealtimeTick();
      });
   }

   public boolean getIsCalibrated()
   {
      return isCalibrated;
   }

   public boolean getNeedsReset()
   {
      return needsReset;
   }

   public double getCurrentTemperature()
   {
      return currentTemperature;
   }

   public double getCurrentHandOpenAngle()
   {
      return currentHandOpenAngle;
   }

   public double getCommandedHandOpenAngle()
   {
      return commandedHandOpenAngle;
   }

   public double getCurrentFingertipGripForce()
   {
      return currentFingertipGripForce;
   }

   public double getCommandedFingertipGripForceLimit()
   {
      return commandedFingertipGripForceLimit;
   }

   public boolean isTorqueOn()
   {
      return isTorqueOn;
   }

   public double getCurrentVelocity()
   {
      return currentVelocity;
   }

   public int getErrorCodes()
   {
      return errorCodes;
   }

   public int getHandRealtimeTick()
   {
      return handRealtimeTick;
   }
}
