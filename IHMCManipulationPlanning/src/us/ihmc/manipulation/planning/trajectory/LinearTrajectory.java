package us.ihmc.manipulation.planning.trajectory;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class LinearTrajectory
{
   private Pose3D initialPose;
   private Pose3D finalPose;
   private double trajectoryTime;

   public LinearTrajectory(Pose3D initialPose, Pose3D finalPose, double trajectoryTime)
   {
      this.initialPose = initialPose;
      this.finalPose = finalPose;
      this.trajectoryTime = trajectoryTime;
   }

   public Pose3D getPose(double time)
   {
      if (time < 0)
         time = 0;
      else if (time > getTrajectoryTime())
         time = getTrajectoryTime();

      double interpolatedScale = time / getTrajectoryTime();

      Point3D initialPoint = new Point3D(initialPose.getPosition());
      Point3D finalPoint = new Point3D(finalPose.getPosition());

      Quaternion startOrientation = new Quaternion(initialPose.getOrientation());
      Quaternion endOrientation = new Quaternion(finalPose.getOrientation());

      double interpolatedPointX = (finalPoint.getX() - initialPoint.getX()) * interpolatedScale + initialPoint.getX();
      double interpolatedPointY = (finalPoint.getY() - initialPoint.getY()) * interpolatedScale + initialPoint.getY();
      double interpolatedPointZ = (finalPoint.getZ() - initialPoint.getZ()) * interpolatedScale + initialPoint.getZ();

      Point3D interpolatedPoint = new Point3D(interpolatedPointX, interpolatedPointY, interpolatedPointZ);
      Quaternion interpolatedOrientation = new Quaternion();

      interpolatedOrientation.interpolate(startOrientation, endOrientation, interpolatedScale);

      Pose3D interpolatedPose = new Pose3D(interpolatedPoint, interpolatedOrientation);

      return interpolatedPose;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }
}
