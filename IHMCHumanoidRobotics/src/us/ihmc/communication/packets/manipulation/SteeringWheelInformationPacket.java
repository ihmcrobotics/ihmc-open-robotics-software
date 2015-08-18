package us.ihmc.communication.packets.manipulation;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class SteeringWheelInformationPacket extends Packet<SteeringWheelInformationPacket>
{
   public RobotSide robotSide;

   public Point3d steeringWheelCenter;
   public Vector3d steeringWheelRotationAxis;
   public Vector3d steeringWheelZeroAxis;

   public double steeringWheelRadius;
   public double desiredSteeringSpeed;
   public double graspOffsetFromControlFrame;
   public int steeringWheelId;

   public SteeringWheelInformationPacket()
   {
   }

   public SteeringWheelInformationPacket(RobotSide robotSide, Point3d steeringWheelCenter, Vector3d steeringWheelRotationAxis, Vector3d steeringWheelZeroAxis,
         double steeringWheelRadius, double graspOffsetFromControlFrame, double desiredSteeringSpeed, int steeringWheelId)
   {
      this.robotSide = robotSide;
      this.steeringWheelCenter = steeringWheelCenter;
      this.steeringWheelRotationAxis = steeringWheelRotationAxis;
      this.steeringWheelZeroAxis = steeringWheelZeroAxis;
      this.steeringWheelRadius = steeringWheelRadius;
      this.graspOffsetFromControlFrame = graspOffsetFromControlFrame;
      this.desiredSteeringSpeed = desiredSteeringSpeed;
      this.steeringWheelId = steeringWheelId;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public Point3d getSteeringWheelCenter()
   {
      return steeringWheelCenter;
   }

   public Vector3d getSteeringWheelRotationAxis()
   {
      return steeringWheelRotationAxis;
   }

   public Vector3d getSteeringWheelZeroAxis()
   {
      return steeringWheelZeroAxis;
   }

   public double getSteeringWheelRadius()
   {
      return steeringWheelRadius;
   }

   public double getGraspOffsetFromControlFrame()
   {
      return graspOffsetFromControlFrame;
   }

   public double getDesiredSteeringSpeed()
   {
      return desiredSteeringSpeed;
   }

   public int getSteeringWheelId()
   {
      return steeringWheelId;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setSteeringWheelCenter(Point3d steeringWheelCenter)
   {
      this.steeringWheelCenter = steeringWheelCenter;
   }

   public void setSteeringWheelRotationAxis(Vector3d steeringWheelRotationAxis)
   {
      this.steeringWheelRotationAxis = steeringWheelRotationAxis;
   }

   public void setSteeringWheelZeroAxis(Vector3d steeringWheelZeroAxis)
   {
      this.steeringWheelZeroAxis = steeringWheelZeroAxis;
   }

   public void setSteeringWheelRadius(double steeringWheelRadius)
   {
      this.steeringWheelRadius = steeringWheelRadius;
   }

   public void setGraspOffsetFromControlFrame(double offset)
   {
      graspOffsetFromControlFrame = offset;
   }

   public void setDesiredSteeringSpeed(double desiredSteeringSpeed)
   {
      this.desiredSteeringSpeed = desiredSteeringSpeed;
   }

   public void setSteeringWheelId(int steeringWheelId)
   {
      this.steeringWheelId = steeringWheelId;
   }

   @Override
   public boolean epsilonEquals(SteeringWheelInformationPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (robotSide == null || other.robotSide == null)
         return false;
      if (steeringWheelCenter == null || other.steeringWheelCenter == null)
         return false;
      if (steeringWheelRotationAxis == null || other.steeringWheelRotationAxis == null)
         return false;
      if (steeringWheelZeroAxis == null || other.steeringWheelZeroAxis == null)
         return false;

      if (robotSide != other.robotSide)
         return false;
      if (!steeringWheelCenter.epsilonEquals(other.steeringWheelCenter, epsilon))
         return false;
      if (!steeringWheelRotationAxis.epsilonEquals(other.steeringWheelRotationAxis, epsilon))
         return false;
      if (!steeringWheelZeroAxis.epsilonEquals(other.steeringWheelZeroAxis, epsilon))
         return false;
      if (!MathTools.epsilonEquals(steeringWheelRadius, other.steeringWheelRadius, epsilon))
         return false;
      if (!MathTools.epsilonEquals(graspOffsetFromControlFrame, other.graspOffsetFromControlFrame, epsilon))
         return false;
      if (!MathTools.epsilonEquals(desiredSteeringSpeed, other.desiredSteeringSpeed, epsilon))
         return false;
      if (steeringWheelId != other.steeringWheelId)
         return false;
      return true;

   }
}
