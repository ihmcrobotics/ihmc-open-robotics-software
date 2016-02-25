package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;

public class SimpleSE3TrajectoryPoint implements SE3TrajectoryPointInterface<SimpleSE3TrajectoryPoint>
{
   private double time = 0.0;
   private final Point3d position = new Point3d();
   private final Quat4d orientation = new Quat4d();
   private final Vector3d linearVelocity = new Vector3d();
   private final Vector3d angularVelocity = new Vector3d();

   public SimpleSE3TrajectoryPoint()
   {
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
   }

   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void setAngularVelocity(Vector3d angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   @Override
   public void set(SimpleSE3TrajectoryPoint other)
   {
      time = other.time;
      position.set(other.position);
      orientation.set(other.orientation);
      linearVelocity.set(other.linearVelocity);
      angularVelocity.set(other.angularVelocity);
   }

   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      time += timeOffsetToAdd;
   }

   @Override
   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time -= timeOffsetToSubtract;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      positionToPack.set(position);
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   @Override
   public boolean epsilonEquals(SimpleSE3TrajectoryPoint other, double epsilon)
   {
      if (!MathTools.epsilonEquals(time, other.time, epsilon))
         return false;
      if (!position.epsilonEquals(other.position, epsilon))
         return false;
      if (!orientation.epsilonEquals(other.orientation, epsilon))
         return false;
      if (!linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;
      if (!angularVelocity.epsilonEquals(other.angularVelocity, epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String xToString = doubleFormat.format(position.getX());
      String yToString = doubleFormat.format(position.getY());
      String zToString = doubleFormat.format(position.getZ());
      String xDotToString = doubleFormat.format(linearVelocity.getX());
      String yDotToString = doubleFormat.format(linearVelocity.getY());
      String zDotToString = doubleFormat.format(linearVelocity.getZ());
      String qxToString = doubleFormat.format(orientation.getX());
      String qyToString = doubleFormat.format(orientation.getY());
      String qzToString = doubleFormat.format(orientation.getZ());
      String qsToString = doubleFormat.format(orientation.getW());
      String wxToString = doubleFormat.format(angularVelocity.getX());
      String wyToString = doubleFormat.format(angularVelocity.getY());
      String wzToString = doubleFormat.format(angularVelocity.getZ());

      String timeToString = "time = " + doubleFormat.format(time);
      String positionToString = "position = (" + xToString + ", " + yToString + ", " + zToString + ")";
      String orientationToString = "orientation = (" + qxToString + ", " + qyToString + ", " + qzToString + ", " + qsToString + ")";
      String linearVelocityToString = "linear velocity = (" + xDotToString + ", " + yDotToString + ", " + zDotToString + ")";
      String angularVelocityToString = "angular velocity = (" + wxToString + ", " + wyToString + ", " + wzToString + ")";

      return "SE3 trajectory point: (" + timeToString + ", " + positionToString + ", " + orientationToString + ", " + linearVelocityToString + ", "
            + angularVelocityToString + ")";
   }
}
