package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanTrajectoryPoint extends FrameTrajectoryPoint<FrameEuclideanTrajectoryPoint>
      implements EuclideanTrajectoryPointInterface<FrameEuclideanTrajectoryPoint>
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   private double time;
   private final FramePoint position = new FramePoint();
   private final FrameVector linearVelocity = new FrameVector();

   public FrameEuclideanTrajectoryPoint()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameEuclideanTrajectoryPoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameEuclideanTrajectoryPoint(double time, FramePoint position, FrameVector linearVelocity)
   {
      setIncludingFrame(time, position, linearVelocity);
   }

   public FrameEuclideanTrajectoryPoint(ReferenceFrame referenceFrame, EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPointInterface)
   {
      setIncludingFrame(referenceFrame, euclideanTrajectoryPointInterface);
   }

   public FrameEuclideanTrajectoryPoint(FrameEuclideanTrajectoryPoint other)
   {
      setIncludingFrame(other);
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

   public void setPosition(FramePoint position)
   {
      this.position.set(position);
   }

   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void setLinearVelocity(FrameVector linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
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

   public void set(EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      // Ensuring frame consistency without crashing
      setIncludingFrame(referenceFrame, euclideanTrajectoryPoint);
   }

   @Override
   public void set(FrameEuclideanTrajectoryPoint other)
   {
      checkReferenceFrameMatch(other);
      other.checkFrameConsistency();

      // Ensuring frame consistency without crashing
      setToZero(referenceFrame);

      time = other.time;
      position.set(other.position);
      linearVelocity.set(other.linearVelocity);
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

   public void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      setToZero(referenceFrame);

      time = euclideanTrajectoryPoint.getTime();
      euclideanTrajectoryPoint.getPosition(position.getPoint());
      euclideanTrajectoryPoint.getLinearVelocity(linearVelocity.getVector());
   }

   @Override
   public void setIncludingFrame(FrameEuclideanTrajectoryPoint other)
   {
      other.checkFrameConsistency();

      referenceFrame = other.referenceFrame;
      time = other.time;
      position.setIncludingFrame(other.position);
      linearVelocity.setIncludingFrame(other.linearVelocity);
   }

   public void setTimeToZero()
   {
      time = 0.0;
   }

   public void setPositionToZero()
   {
      position.setToZero();
   }

   public void setLinearVelocityToZero()
   {
      linearVelocity.setToZero();
   }

   @Override
   public void setToZero()
   {
      time = 0.0;
      position.setToZero();
      linearVelocity.setToZero();
   }

   @Override
   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      time = 0.0;
      position.setToZero(referenceFrame);
      linearVelocity.setToZero(referenceFrame);
   }

   public void setTimeToNaN()
   {
      time = Double.NaN;
   }

   public void setPositionToNaN()
   {
      position.setToNaN();
   }

   public void setLinearVelocityToNaN()
   {
      linearVelocity.setToNaN();
   }

   @Override
   public void setToNaN()
   {
      time = Double.NaN;
      position.setToNaN();
      linearVelocity.setToNaN();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      time = Double.NaN;
      position.setToNaN(referenceFrame);
      linearVelocity.setToNaN(referenceFrame);
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
   public boolean containsNaN()
   {
      return Double.isNaN(time) || position.containsNaN() || linearVelocity.containsNaN();
   }

   public void checkFrameConsistency()
   {
      if (referenceFrame != position.getReferenceFrame())
         throwFrameInconsistencyException();
      if (referenceFrame != linearVelocity.getReferenceFrame())
         throwFrameInconsistencyException();
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
    * Return the original position held by this trajectory point.
    */
   public FramePoint getPosition()
   {
      return position;
   }

   /**
    * Return the original linearVelocity held by this trajectory point.
    */
   public FrameVector getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
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

   @Override
   public boolean epsilonEquals(FrameEuclideanTrajectoryPoint other, double epsilon)
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
      String xToString = doubleFormat.format(position.getX());
      String yToString = doubleFormat.format(position.getY());
      String zToString = doubleFormat.format(position.getZ());
      String xDotToString = doubleFormat.format(linearVelocity.getX());
      String yDotToString = doubleFormat.format(linearVelocity.getY());
      String zDotToString = doubleFormat.format(linearVelocity.getZ());

      String timeToString = "time = " + doubleFormat.format(time);
      String positionToString = "position = (" + xToString + ", " + yToString + ", " + zToString + ")";
      String linearVelocityToString = "linear velocity = (" + xDotToString + ", " + yDotToString + ", " + zDotToString + ")";
      String referenceFrameToString = "reference frame = " + referenceFrame.getName();

      return "Euclidean trajectory point: (" + timeToString + ", " + positionToString + ", " + linearVelocityToString + ", " + referenceFrameToString + ")";
   }
}
