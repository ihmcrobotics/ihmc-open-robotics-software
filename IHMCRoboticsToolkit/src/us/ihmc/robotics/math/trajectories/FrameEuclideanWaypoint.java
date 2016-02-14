package us.ihmc.robotics.math.trajectories;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanWaypoint extends ReferenceFrameHolder implements EuclideanWaypointInterface
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   private double time;
   private final FramePoint position = new FramePoint();
   private final FrameVector linearVelocity = new FrameVector();

   public FrameEuclideanWaypoint()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameEuclideanWaypoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameEuclideanWaypoint(double time, FramePoint position, FrameVector linearVelocity)
   {
      setIncludingFrame(time, position, linearVelocity);
   }

   public FrameEuclideanWaypoint(ReferenceFrame referenceFrame, EuclideanWaypointInterface euclideanWaypointInterface)
   {
      setIncludingFrame(referenceFrame, euclideanWaypointInterface);
   }

   public FrameEuclideanWaypoint(FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      setIncludingFrame(frameEuclideanWaypoint);
   }

   public void set(double time, Point3d position, Vector3d linearVelocity)
   {
      // Ensuring frame consistency without crashing
      setIncludingFrame(referenceFrame, time, position, linearVelocity);
   }

   public void set(double time, FramePoint position, FrameVector linearVelocity)
   {
      // Ensuring frame consistency without crashing
      setToZero(referenceFrame);
      this.time = time;
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   public void set(EuclideanWaypointInterface euclideanWaypoint)
   {
      // Ensuring frame consistency without crashing
      setIncludingFrame(referenceFrame, euclideanWaypoint);
   }

   public void set(FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      checkReferenceFrameMatch(frameEuclideanWaypoint);
      frameEuclideanWaypoint.checkFrameConsistency();

      // Ensuring frame consistency without crashing
      setToZero(referenceFrame);

      time = frameEuclideanWaypoint.time;
      position.set(frameEuclideanWaypoint.position);
      linearVelocity.set(frameEuclideanWaypoint.linearVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3d position, Vector3d linearVelocity)
   {
      setToZero(referenceFrame);

      this.time = time;
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   public void setIncludingFrame(double time, FramePoint position, FrameVector linearVelocity)
   {
      position.checkReferenceFrameMatch(linearVelocity);
      referenceFrame = position.getReferenceFrame();

      this.time = time;
      this.position.setIncludingFrame(position);
      this.linearVelocity.setIncludingFrame(linearVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanWaypointInterface euclideanWaypoint)
   {
      setToZero(referenceFrame);

      time = euclideanWaypoint.getTime();
      euclideanWaypoint.getPosition(position.getPoint());
      euclideanWaypoint.getLinearVelocity(linearVelocity.getVector());
   }

   public void setIncludingFrame(FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      frameEuclideanWaypoint.checkFrameConsistency();

      referenceFrame = frameEuclideanWaypoint.referenceFrame;
      time = frameEuclideanWaypoint.time;
      position.setIncludingFrame(frameEuclideanWaypoint.position);
      linearVelocity.setIncludingFrame(frameEuclideanWaypoint.linearVelocity);
   }

   public void setToZero()
   {
      time = 0.0;
      position.setToZero();
      linearVelocity.setToZero();
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      time = 0.0;
      position.setToZero(referenceFrame);
      linearVelocity.setToZero(referenceFrame);
   }

   public void setToNaN()
   {
      time = Double.NaN;
      position.setToNaN();
      linearVelocity.setToNaN();
   }

   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      time = Double.NaN;
      position.setToNaN(referenceFrame);
      linearVelocity.setToNaN(referenceFrame);
   }

   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time -= timeOffsetToSubtract;
   }

   public void checkFrameConsistency()
   {
      if (referenceFrame != position.getReferenceFrame())
         throwFrameInconsistencyException();
      if (referenceFrame != linearVelocity.getReferenceFrame())
         throwFrameInconsistencyException();
   }

   private void throwFrameInconsistencyException()
   {
      throw new RuntimeException("The reference frames in the " + getClass().getSimpleName() + " are inconsistent.");
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      position.get(positionToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocity.get(linearVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.set(position);
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(linearVelocity);
   }

   /**
    * Return the original position held by this waypoint.
    */
   public FramePoint getPosition()
   {
      return position;
   }

   /**
    * Return the original linearVelocity held by this waypoint.
    */
   public FrameVector getLinearVelocity()
   {
      return linearVelocity;
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      position.changeFrame(referenceFrame);
      linearVelocity.changeFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public boolean epsilonEquals(FrameEuclideanWaypoint other, double epsilon)
   {
      checkFrameConsistency();
      other.checkFrameConsistency();

      if (referenceFrame != other.referenceFrame)
         return false;
      if (!MathTools.epsilonEquals(time, other.time, epsilon))
         return false;
      if (!position.epsilonEquals(other.position, epsilon))
         return false;
      if (!linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(time);
      String positionToString = ", position = (" + doubleFormat.format(position.getX()) + ", " + doubleFormat.format(position.getY()) + ", " + doubleFormat.format(position.getZ()) + ")";
      String linearVelocityToString = ", linear velocity = (" + doubleFormat.format(linearVelocity.getX()) + ", " + doubleFormat.format(linearVelocity.getY()) + ", " + doubleFormat.format(linearVelocity.getZ()) + ")";
      String referenceFrameToString = ", reference frame = " + getReferenceFrame();

      return "(" + timeToString + positionToString + linearVelocityToString + referenceFrameToString + ")";
   }
}
