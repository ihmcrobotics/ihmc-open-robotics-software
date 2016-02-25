package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSE3Waypoint extends FrameWaypoint<FrameSE3Waypoint> implements SE3WaypointInterface<FrameSE3Waypoint>
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   private double time;
   private final FramePoint position = new FramePoint();
   private final FrameOrientation orientation = new FrameOrientation();
   private final FrameVector linearVelocity = new FrameVector();
   private final FrameVector angularVelocity = new FrameVector();

   public FrameSE3Waypoint()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameSE3Waypoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameSE3Waypoint(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      setIncludingFrame(time, position, orientation, linearVelocity, angularVelocity);
   }

   public FrameSE3Waypoint(ReferenceFrame referenceFrame, SE3WaypointInterface<?> se3WaypointInterface)
   {
      setIncludingFrame(referenceFrame, se3WaypointInterface);
   }

   public FrameSE3Waypoint(FrameSE3Waypoint frameSE3Waypoint)
   {
      setIncludingFrame(frameSE3Waypoint);
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

   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      this.orientation.set(orientation);
   }

   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void setLinearVelocity(FrameVector linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void setAngularVelocity(Vector3d angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void setAngularVelocity(FrameVector angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      // Ensuring frame consistency without crashing
      setIncludingFrame(referenceFrame, time, position, orientation, linearVelocity, angularVelocity);
   }

   public void set(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      // Ensuring frame consistency without crashing
      setToZero(referenceFrame);
      this.time = time;
      this.position.set(position);
      this.orientation.set(orientation);
      this.linearVelocity.set(linearVelocity);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(SE3WaypointInterface<?> se3Waypoint)
   {
      // Ensuring frame consistency without crashing
      setIncludingFrame(referenceFrame, se3Waypoint);
   }

   @Override
   public void set(FrameSE3Waypoint frameSE3Waypoint)
   {
      checkReferenceFrameMatch(frameSE3Waypoint);
      frameSE3Waypoint.checkFrameConsistency();

      // Ensuring frame consistency without crashing
      setToZero(referenceFrame);

      time = frameSE3Waypoint.time;
      position.set(frameSE3Waypoint.position);
      orientation.set(frameSE3Waypoint.orientation);
      linearVelocity.set(frameSE3Waypoint.linearVelocity);
      angularVelocity.set(frameSE3Waypoint.angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3d position, Quat4d orientation, Vector3d linearVelocity,
         Vector3d angularVelocity)
   {
      setToZero(referenceFrame);

      this.time = time;
      this.position.set(position);
      this.orientation.set(orientation);
      this.linearVelocity.set(linearVelocity);
      this.angularVelocity.set(angularVelocity);
   }

   public void setIncludingFrame(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      position.checkReferenceFrameMatch(orientation);
      position.checkReferenceFrameMatch(linearVelocity);
      position.checkReferenceFrameMatch(angularVelocity);
      referenceFrame = position.getReferenceFrame();

      this.time = time;
      this.position.setIncludingFrame(position);
      this.orientation.setIncludingFrame(orientation);
      this.linearVelocity.setIncludingFrame(linearVelocity);
      this.angularVelocity.setIncludingFrame(angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SE3WaypointInterface<?> se3Waypoint)
   {
      setToZero(referenceFrame);

      time = se3Waypoint.getTime();
      se3Waypoint.getPosition(position.getPoint());
      se3Waypoint.getOrientation(orientation.getQuaternion());
      se3Waypoint.getLinearVelocity(linearVelocity.getVector());
      se3Waypoint.getAngularVelocity(angularVelocity.getVector());
   }

   @Override
   public void setIncludingFrame(FrameSE3Waypoint frameSE3Waypoint)
   {
      frameSE3Waypoint.checkFrameConsistency();

      referenceFrame = frameSE3Waypoint.referenceFrame;
      time = frameSE3Waypoint.time;
      position.setIncludingFrame(frameSE3Waypoint.position);
      orientation.setIncludingFrame(frameSE3Waypoint.orientation);
      linearVelocity.setIncludingFrame(frameSE3Waypoint.linearVelocity);
      angularVelocity.setIncludingFrame(frameSE3Waypoint.angularVelocity);
   }

   @Override
   public void setToZero()
   {
      time = 0.0;
      position.setToZero();
      orientation.setToZero();
      linearVelocity.setToZero();
      angularVelocity.setToZero();
   }

   @Override
   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      time = 0.0;
      position.setToZero(referenceFrame);
      orientation.setToZero(referenceFrame);
      linearVelocity.setToZero(referenceFrame);
      angularVelocity.setToZero(referenceFrame);
   }

   @Override
   public void setToNaN()
   {
      time = Double.NaN;
      position.setToNaN();
      orientation.setToNaN();
      linearVelocity.setToNaN();
      angularVelocity.setToNaN();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      time = Double.NaN;
      position.setToNaN(referenceFrame);
      orientation.setToNaN(referenceFrame);
      linearVelocity.setToNaN(referenceFrame);
      angularVelocity.setToNaN(referenceFrame);
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

   public void checkFrameConsistency()
   {
      if (referenceFrame != position.getReferenceFrame())
         throwFrameInconsistencyException();
      if (referenceFrame != orientation.getReferenceFrame())
         throwFrameInconsistencyException();
      if (referenceFrame != linearVelocity.getReferenceFrame())
         throwFrameInconsistencyException();
      if (referenceFrame != angularVelocity.getReferenceFrame())
         throwFrameInconsistencyException();
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(time) || position.containsNaN() || orientation.containsNaN() || linearVelocity.containsNaN() || angularVelocity.containsNaN();
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
   public void getOrientation(Quat4d orientationToPack)
   {
      orientation.getQuaternion(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocity.get(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.set(position);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(linearVelocity);
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(angularVelocity);
   }

   /**
    * Return the original position held by this waypoint.
    */
   public FramePoint getPosition()
   {
      return position;
   }

   /**
    * Return the original orientation held by this waypoint.
    */
   public FrameOrientation getOrientation()
   {
      return orientation;
   }

   /**
    * Return the original linearVelocity held by this waypoint.
    */
   public FrameVector getLinearVelocity()
   {
      return linearVelocity;
   }

   /**
    * Return the original angularVelocity held by this waypoint.
    */
   public FrameVector getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public void changeFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      position.changeFrame(referenceFrame);
      orientation.changeFrame(referenceFrame);
      linearVelocity.changeFrame(referenceFrame);
      angularVelocity.changeFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public boolean epsilonEquals(FrameSE3Waypoint other, double epsilon)
   {
      checkFrameConsistency();
      other.checkFrameConsistency();

      if (referenceFrame != other.referenceFrame)
         return false;
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
      String qxToString = doubleFormat.format(orientation.getQx());
      String qyToString = doubleFormat.format(orientation.getQy());
      String qzToString = doubleFormat.format(orientation.getQz());
      String qsToString = doubleFormat.format(orientation.getQs());
      String wxToString = doubleFormat.format(angularVelocity.getX());
      String wyToString = doubleFormat.format(angularVelocity.getY());
      String wzToString = doubleFormat.format(angularVelocity.getZ());

      String timeToString = "time = " + doubleFormat.format(time);
      String positionToString = "position = (" + xToString + ", " + yToString + ", " + zToString + ")";
      String orientationToString = "orientation = (" + qxToString + ", " + qyToString + ", " + qzToString + ", " + qsToString + ")";
      String linearVelocityToString = "linear velocity = (" + xDotToString + ", " + yDotToString + ", " + zDotToString + ")";
      String angularVelocityToString = "angular velocity = (" + wxToString + ", " + wyToString + ", " + wzToString + ")";
      String referenceFrameToString = "reference frame = " + getReferenceFrame();

      return "SE3 waypoint: (" + timeToString + ", " + positionToString + ", " + orientationToString + ", " + linearVelocityToString + ", "
            + angularVelocityToString + ", " + referenceFrameToString + ")";
   }
}
