package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class SteeringWheelInformationPacket extends Packet<SteeringWheelInformationPacket>
{
   public RobotSide robotSide;

   public Point3D steeringWheelCenter;
   public Vector3D steeringWheelRotationAxis;
   public Vector3D steeringWheelZeroAxis;

   public double steeringWheelRadius;
   public double desiredSteeringSpeed;
   public double graspOffsetFromControlFrame;
   public int steeringWheelId;

   public SteeringWheelInformationPacket()
   {
   }

   public SteeringWheelInformationPacket(RobotSide robotSide, Point3D steeringWheelCenter, Vector3D steeringWheelRotationAxis, Vector3D steeringWheelZeroAxis,
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

   public Point3D getSteeringWheelCenter()
   {
      return steeringWheelCenter;
   }

   public Vector3D getSteeringWheelRotationAxis()
   {
      return steeringWheelRotationAxis;
   }

   public Vector3D getSteeringWheelZeroAxis()
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

   public void setSteeringWheelCenter(Point3D steeringWheelCenter)
   {
      this.steeringWheelCenter = steeringWheelCenter;
   }

   public void setSteeringWheelRotationAxis(Vector3D steeringWheelRotationAxis)
   {
      this.steeringWheelRotationAxis = steeringWheelRotationAxis;
   }

   public void setSteeringWheelZeroAxis(Vector3D steeringWheelZeroAxis)
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
