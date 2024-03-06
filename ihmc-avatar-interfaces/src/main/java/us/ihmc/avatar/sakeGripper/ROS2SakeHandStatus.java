package us.ihmc.avatar.sakeGripper;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;

public class ROS2SakeHandStatus
{
   private volatile boolean isCalibrated = false;
   private volatile boolean needsReset;
   private volatile double currentTemperature = Double.NaN;
   private volatile double currentHandOpenAngle = Double.NaN;
   private volatile double commandedHandOpenAngle = Double.NaN;
   private volatile double currentFingertipGripForce = Double.NaN;
   private volatile double commandedFingertipGripForceLimit = Double.NaN;

   public ROS2SakeHandStatus(ROS2NodeInterface ros2Node, String robotName, RobotSide handSide)
   {
      ROS2Tools.createVolatileCallbackSubscription(ros2Node, ROS2Tools.getHandSakeStatusTopic(robotName, handSide), sakeHandStatusMessage ->
      {
         isCalibrated = sakeHandStatusMessage.getIsCalibrated();
         needsReset = sakeHandStatusMessage.getNeedsReset();
         currentTemperature = sakeHandStatusMessage.getTemperature();
         currentHandOpenAngle = SakeHandParameters.denormalizeHandOpenAngle(sakeHandStatusMessage.getNormalizedCurrentPosition());
         commandedHandOpenAngle = SakeHandParameters.denormalizeHandOpenAngle(sakeHandStatusMessage.getNormalizedDesiredPosition());
         currentFingertipGripForce = SakeHandParameters.denormalizeFingertipGripForceLimit(sakeHandStatusMessage.getNormalizedCurrentTorque());
         commandedFingertipGripForceLimit = SakeHandParameters.denormalizeFingertipGripForceLimit(sakeHandStatusMessage.getNormalizedTorqueLimit());
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
}
