package us.ihmc.avatar.sakeGripper;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.SakeHandAPI;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.robotics.robotSide.RobotSide;

public class ROS2SakeHandStatus
{
   private volatile boolean isCalibrated = false;
   private volatile boolean needsReset;
   private volatile boolean isCalibrating;
   private volatile boolean isCoolingDown;
   private volatile boolean automaticCooldownEnabled;

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

   public ROS2SakeHandStatus(ROS2Node ros2Node, String robotName, RobotSide handSide)
   {
      var sakeHandStatusTopic = SakeHandAPI.getHandSakeStatusTopic(robotName, handSide);
      ros2Node.createSubscriptionSampler(sakeHandStatusTopic, sample ->
      {
         if (sample == null)
            return;
         isCalibrated = sample.getIsCalibrated();
         needsReset = sample.getNeedsReset();
         isCalibrating = sample.getIsCalibrating();
         isCoolingDown = sample.getIsCoolingDown();
         automaticCooldownEnabled = sample.getAutomaticCooldownEnabled();

         positionUpperLimit = sample.getPositionUpperLimit();
         positionLowerLimit = sample.getPositionLowerLimit();

         isTorqueOn = sample.getTorqueOnStatus();
         currentTemperature = sample.getTemperature();
         currentHandOpenAngle = SakeHandParameters.handPositionToOpenAngle(sample.getCurrentPosition(),
                                                                           positionLowerLimit,
                                                                           positionUpperLimit);
         commandedHandOpenAngle = SakeHandParameters.handPositionToOpenAngle(sample.getDesiredPositionStatus(),
                                                                             positionLowerLimit,
                                                                             positionUpperLimit);
         commandedHandOpenAngle = MathTools.clamp(commandedHandOpenAngle, 0.0, SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES);
         currentFingertipGripForce = sample.getRawCurrentTorque() * SakeHandParameters.RAW_SAKE_TORQUE_TO_GRIP_FORCE;
         commandedFingertipGripForceLimit = sample.getRawTorqueLimitStatus() * SakeHandParameters.RAW_SAKE_TORQUE_TO_GRIP_FORCE;
         currentVelocity = sample.getCurrentVelocity();
         errorCodes = sample.getErrorCodes();
         handRealtimeTick = sample.getRealtimeTick();
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

   public boolean getIsCalibrating()
   {
      return isCalibrating;
   }

   public boolean getIsCoolingDown()
   {
      return isCoolingDown;
   }

   public boolean getAutomaticCooldownEnabled()
   {
      return automaticCooldownEnabled;
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
