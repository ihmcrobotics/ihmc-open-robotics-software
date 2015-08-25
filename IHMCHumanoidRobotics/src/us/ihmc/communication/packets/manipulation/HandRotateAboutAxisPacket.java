package us.ihmc.communication.packets.manipulation;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandRotateAboutAxisPacket extends Packet<HandRotateAboutAxisPacket>
{
   public RobotSide robotSide;

   @FieldDocumentation("trajectoryTime specifies how fast to move the hand to the desired position")
   public double trajectoryTime;

   public Point3d rotationAxisOriginInWorld;
   public Vector3d rotationAxisInWorld;
   public double rotationRightHandRule;
   public boolean controlHandAngleAboutAxis;
   public double graspOffsetFromControlFrame;
   
   public enum DataType{ROTATE_ABOUT_AXIS_FORCE_CONTROLLED}
   public DataType dataType = null;
   
   //Fields for force control, only used when DataType != null
   public double desiredTangentialForce;
   public Vector3d forceConstraint;

   public HandRotateAboutAxisPacket()
   {
      // Empty constructor for deserialization
   }

   public HandRotateAboutAxisPacket(RobotSide robotSide, Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld, double rotationAngleRightHandRule,
         double trajectoryTime, boolean controlHandAngleAboutAxis)
   {
      this(robotSide, rotationAxisOriginInWorld, rotationAxisInWorld, rotationAngleRightHandRule, trajectoryTime, 0.0, controlHandAngleAboutAxis);
   }

   public HandRotateAboutAxisPacket(RobotSide robotSide, Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld, double rotationAngleRightHandRule,
         double trajectoryTime, double graspOffsetFromControlFrame, boolean controlHandAngleAboutAxis)
   {
      this.setDestination(PacketDestination.CONTROLLER);
      this.robotSide = robotSide;
      this.rotationAxisOriginInWorld = rotationAxisOriginInWorld;
      this.rotationAxisInWorld = rotationAxisInWorld;
      this.rotationRightHandRule = rotationAngleRightHandRule;
      this.trajectoryTime = trajectoryTime;
      this.graspOffsetFromControlFrame = graspOffsetFromControlFrame;
      this.controlHandAngleAboutAxis = controlHandAngleAboutAxis;
   }
   
   public void setForceControlParameters(double desiredTangentialForce, Vector3d forceConstraint)
   {
      dataType = DataType.ROTATE_ABOUT_AXIS_FORCE_CONTROLLED;
      this.desiredTangentialForce = desiredTangentialForce;
      this.forceConstraint = forceConstraint;
   }

   public DataType getDataType()
   {
      return dataType;
   }
   
   public double getTangentialForce()
   {
	   if(dataType == DataType.ROTATE_ABOUT_AXIS_FORCE_CONTROLLED)
	   {
		   return desiredTangentialForce;
	   }
	   else
	   {
		   return Double.NaN;
	   }
   }
   
   public Vector3d getForceConstraint()
   {
	   return forceConstraint;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public Point3d getRotationAxisOriginInWorld()
   {
      return rotationAxisOriginInWorld;
   }

   public Vector3d getRotationAxisInWorld()
   {
      return rotationAxisInWorld;
   }

   public double getRotationAngleRightHandRule()
   {
      return rotationRightHandRule;
   }

   public boolean controlHandAngleAboutAxis()
   {
      return controlHandAngleAboutAxis;
   }

   public double getGraspOffsetFromControlFrame()
   {
      return graspOffsetFromControlFrame;
   }

   @Override
   public boolean epsilonEquals(HandRotateAboutAxisPacket other, double epsilon)
   {
      boolean sameRobotSide = other.robotSide == robotSide;
      boolean sameTrajectoryTime = other.trajectoryTime == trajectoryTime;
      boolean sameRotationAxisOrigin = other.rotationAxisOriginInWorld == rotationAxisOriginInWorld;
      boolean sameRotationAxis = other.rotationAxisInWorld == rotationAxisInWorld;
      boolean sameRotationAngle = other.rotationRightHandRule == rotationRightHandRule;
      boolean sameOrientationLockBoolean = other.controlHandAngleAboutAxis == controlHandAngleAboutAxis;
      
      boolean sameDesiredTangentialForce;
      boolean sameForceConstraint;
      
      if(dataType != null)
      {
    	  sameDesiredTangentialForce = MathTools.epsilonEquals(other.desiredTangentialForce, desiredTangentialForce, epsilon);
    	  sameForceConstraint = forceConstraint.epsilonEquals(other.forceConstraint, epsilon);
      }
      else
      {
    	  sameDesiredTangentialForce = true;
    	  sameForceConstraint = true;
      }
      
      return sameRobotSide && sameTrajectoryTime && sameRotationAxisOrigin && sameRotationAxis && sameRotationAngle && sameOrientationLockBoolean && sameDesiredTangentialForce && sameForceConstraint;
   }
}
