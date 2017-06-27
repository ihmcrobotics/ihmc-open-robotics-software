package us.ihmc.manipulation.planning.trajectory;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.transformables.Pose;

public class LinearTrajectory
{
   private Pose initialPose;
   private Pose finalPose;
   private double trajectoryTime;
   
   public LinearTrajectory(Pose initialPose, Pose finalPose, double trajectoryTime)
   {
      this.initialPose = initialPose;
      this.finalPose = finalPose;
      this.trajectoryTime = trajectoryTime;
   }
   
   public Pose getPose(double time)
   {        
      if(time < 0)
         time = 0;
      else if(time > getTrajectoryTime())
         time = getTrajectoryTime();
      
      double interpolatedScale = time/getTrajectoryTime();
      
      Point3D initialPoint = new Point3D(initialPose.getPoint());
      Point3D finalPoint = new Point3D(finalPose.getPoint());
      
      Quaternion startOrientation = new Quaternion(initialPose.getOrientation());
      Quaternion endOrientation = new Quaternion(finalPose.getOrientation());
      
      double interpolatedPointX = (finalPoint.getX() - initialPoint.getX())*interpolatedScale + initialPoint.getX();
      double interpolatedPointY = (finalPoint.getY() - initialPoint.getY())*interpolatedScale + initialPoint.getY();
      double interpolatedPointZ = (finalPoint.getZ() - initialPoint.getZ())*interpolatedScale + initialPoint.getZ();
            
      Point3D interpolatedPoint = new Point3D(interpolatedPointX, interpolatedPointY, interpolatedPointZ);
      Quaternion interpolatedOrientation = new Quaternion();
      
      interpolatedOrientation.interpolate(startOrientation, endOrientation, interpolatedScale);
      
      Pose interpolatedPose = new Pose(interpolatedPoint, interpolatedOrientation);
      
      return interpolatedPose;
   }
   
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }
}
