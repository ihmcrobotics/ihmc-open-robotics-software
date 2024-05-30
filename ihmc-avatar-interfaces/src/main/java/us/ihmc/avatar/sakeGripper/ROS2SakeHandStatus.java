package us.ihmc.avatar.sakeGripper;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.SakeHandAPI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;

public class ROS2SakeHandStatus
{
   private volatile boolean isCalibrated = false;
   private volatile boolean needsReset;

   // Hand fully closed
   private volatile double positionUpperLimit;
   // Hand fully open
   private volatile double positionLowerLimit;

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
      ROS2Tools.createVolatileCallbackSubscription(ros2Node, SakeHandAPI.getHandSakeStatusTopic(robotName, handSide), sakeHandStatusMessage ->
      {
         isCalibrated = sakeHandStatusMessage.getIsCalibrated();
         needsReset = sakeHandStatusMessage.getNeedsReset();

         positionUpperLimit = sakeHandStatusMessage.getPositionUpperLimit();
         positionLowerLimit = sakeHandStatusMessage.getPositionLowerLimit();

         isTorqueOn = sakeHandStatusMessage.getTorqueOnStatus();
         currentTemperature = sakeHandStatusMessage.getTemperature();
         currentHandOpenAngle = SakeHandParameters.handPositionToOpenAngle(sakeHandStatusMessage.getCurrentPosition(),
                                                                           positionLowerLimit,
                                                                           positionUpperLimit);
         commandedHandOpenAngle = SakeHandParameters.handPositionToOpenAngle(sakeHandStatusMessage.getDesiredPositionStatus(),
                                                                             positionLowerLimit,
                                                                             positionUpperLimit);
         commandedHandOpenAngle = MathTools.clamp(commandedHandOpenAngle, 0.0, SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES);
         currentFingertipGripForce = sakeHandStatusMessage.getRawCurrentTorque() * SakeHandParameters.RAW_SAKE_TORQUE_TO_GRIP_FORCE;
         commandedFingertipGripForceLimit = sakeHandStatusMessage.getRawTorqueLimitStatus() * SakeHandParameters.RAW_SAKE_TORQUE_TO_GRIP_FORCE;
         currentVelocity = sakeHandStatusMessage.getCurrentVelocity();
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

   public double getPositionUpperLimit()
   {
      return positionUpperLimit;
   }

   public double getPositionLowerLimit()
   {
      return positionLowerLimit;
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

   public String getErrorString()
   {
      StringBuilder errorString = new StringBuilder();
      for (String errorName : SakeHandError.getErrorNames(errorCodes))
         errorString.append("[").append(errorName).append("]");

      return errorString.toString();
   }

   public int getHandRealtimeTick()
   {
      return handRealtimeTick;
   }
}
