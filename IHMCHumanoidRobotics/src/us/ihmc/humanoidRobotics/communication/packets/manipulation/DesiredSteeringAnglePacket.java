package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class DesiredSteeringAnglePacket extends Packet<DesiredSteeringAnglePacket>
{
   public boolean isRightHandSteering;
   public double desiredAbsoluteSteeringAngle;
   public int steeringWheelId;

   public DesiredSteeringAnglePacket()
   {
   }

   public DesiredSteeringAnglePacket(RobotSide robotSide, double desiredAbsoluteSteeringAngle, int steeringWheelId)
   {
      setRobotSide(robotSide);
      this.desiredAbsoluteSteeringAngle = desiredAbsoluteSteeringAngle;
      this.steeringWheelId = steeringWheelId;
   }

   public RobotSide getRobotSide()
   {
      return isRightHandSteering ? RobotSide.RIGHT : RobotSide.LEFT;
   }

   public double getDesiredAbsoluteSteeringAngle()
   {
      return desiredAbsoluteSteeringAngle;
   }
   
   public int getSteeringWheelId()
   {
      return steeringWheelId;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      if (robotSide == RobotSide.RIGHT)
         isRightHandSteering = true;
      else
         isRightHandSteering = false;
   }

   public void setDesiredAbsoluteSteeringAngle(double desiredAbsoluteSteeringAngle)
   {
      this.desiredAbsoluteSteeringAngle = desiredAbsoluteSteeringAngle;
   }

   public void setSteeringWheelId(int steeringWheelId)
   {
      this.steeringWheelId = steeringWheelId;
   }

   @Override
   public boolean epsilonEquals(DesiredSteeringAnglePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (isRightHandSteering != other.isRightHandSteering)
         return false;
      if (!MathTools.epsilonEquals(desiredAbsoluteSteeringAngle, other.desiredAbsoluteSteeringAngle, epsilon))
         return false;
      if (steeringWheelId != other.steeringWheelId)
         return false;
      return true;
   }
}
