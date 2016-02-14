package us.ihmc.robotics.math.trajectories;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSE3Waypoint extends ReferenceFrameHolder implements SE3WaypointInterface
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

   public FrameSE3Waypoint(ReferenceFrame referenceFrame, SE3WaypointInterface se3WaypointInterface)
   {
      setIncludingFrame(referenceFrame, se3WaypointInterface);
   }

   public FrameSE3Waypoint(FrameSE3Waypoint frameSE3Waypoint)
   {
      setIncludingFrame(frameSE3Waypoint);
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

   public void set(SE3WaypointInterface se3Waypoint)
   {
      // Ensuring frame consistency without crashing
      setIncludingFrame(referenceFrame, se3Waypoint);
   }

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

   public void setIncludingFrame(ReferenceFrame referenceFrame, SE3WaypointInterface se3Waypoint)
   {
      setToZero(referenceFrame);

      time = se3Waypoint.getTime();
      se3Waypoint.getPosition(position.getPoint());
      se3Waypoint.getOrientation(orientation.getQuaternion());
      se3Waypoint.getLinearVelocity(linearVelocity.getVector());
      se3Waypoint.getAngularVelocity(angularVelocity.getVector());
   }

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

   public void setToZero()
   {
      time = 0.0;
      position.setToZero();
      orientation.setToZero();
      linearVelocity.setToZero();
      angularVelocity.setToZero();
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      time = 0.0;
      position.setToZero(referenceFrame);
      orientation.setToZero(referenceFrame);
      linearVelocity.setToZero(referenceFrame);
      angularVelocity.setToZero(referenceFrame);
   }

   public void setToNaN()
   {
      time = Double.NaN;
      position.setToNaN();
      orientation.setToNaN();
      linearVelocity.setToNaN();
      angularVelocity.setToNaN();
   }

   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      time = Double.NaN;
      position.setToNaN(referenceFrame);
      orientation.setToNaN(referenceFrame);
      linearVelocity.setToNaN(referenceFrame);
      angularVelocity.setToNaN(referenceFrame);
   }

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
      String timeToString = "time = " + doubleFormat.format(time);
      String positionToString = ", position = (" + doubleFormat.format(position.getX()) + ", " + doubleFormat.format(position.getY()) + ", " + doubleFormat.format(position.getZ()) + ")";
      String orientationToString = ", orientation = (" + doubleFormat.format(orientation.getQx()) + ", " + doubleFormat.format(orientation.getQy()) + ", " + doubleFormat.format(orientation.getQz()) + ", " + doubleFormat.format(orientation.getQs()) + ")";
      String linearVelocityToString = ", linear velocity = (" + doubleFormat.format(linearVelocity.getX()) + ", " + doubleFormat.format(linearVelocity.getY()) + ", " + doubleFormat.format(linearVelocity.getZ()) + ")";
      String angularVelocityToString = ", angular velocity = (" + doubleFormat.format(angularVelocity.getX()) + ", " + doubleFormat.format(angularVelocity.getY()) + ", " + doubleFormat.format(angularVelocity.getZ()) + ")";
      String referenceFrameToString = ", reference frame = " + getReferenceFrame();

      return "(" + timeToString + positionToString + orientationToString + linearVelocityToString + angularVelocityToString + referenceFrameToString + ")";
   }
}
