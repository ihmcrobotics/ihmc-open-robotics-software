package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSE3TrajectoryPoint extends FrameTrajectoryPoint<FrameSE3TrajectoryPoint, SimpleSE3TrajectoryPoint>
      implements SE3TrajectoryPointInterface<FrameSE3TrajectoryPoint>
{
   public FrameSE3TrajectoryPoint()
   {
      super(new SimpleSE3TrajectoryPoint());
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameSE3TrajectoryPoint(ReferenceFrame referenceFrame)
   {
      this();
      setToZero(referenceFrame);
   }

   public FrameSE3TrajectoryPoint(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      this();
      setIncludingFrame(time, position, orientation, linearVelocity, angularVelocity);
   }

   public FrameSE3TrajectoryPoint(ReferenceFrame referenceFrame, SE3TrajectoryPointInterface<?> se3TrajectoryPointInterface)
   {
      this();
      setIncludingFrame(referenceFrame, se3TrajectoryPointInterface);
   }

   public FrameSE3TrajectoryPoint(FrameSE3TrajectoryPoint frameSE3TrajectoryPoint)
   {
      this();
      setIncludingFrame(frameSE3TrajectoryPoint);
   }

   @Override
   public void setPosition(Point3d position)
   {
      simpleWaypoint.setPosition(position);
   }

   public void setPosition(FramePoint position)
   {
      checkReferenceFrameMatch(position);
      simpleWaypoint.setPosition(position.getPoint());
   }

   @Override
   public void setOrientation(Quat4d orientation)
   {
      simpleWaypoint.setOrientation(orientation);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      checkReferenceFrameMatch(orientation);
      simpleWaypoint.setOrientation(orientation.getQuaternion());
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      simpleWaypoint.setLinearVelocity(linearVelocity);
   }

   public void setLinearVelocity(FrameVector linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      simpleWaypoint.setLinearVelocity(linearVelocity.getVector());
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      simpleWaypoint.setAngularVelocity(angularVelocity);
   }

   public void setAngularVelocity(FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      simpleWaypoint.setAngularVelocity(angularVelocity.getVector());
   }

   public void set(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      simpleWaypoint.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public void set(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(linearVelocity);
      checkReferenceFrameMatch(angularVelocity);
      simpleWaypoint.set(time, position.getPoint(), orientation.getQuaternion(), linearVelocity.getVector(), angularVelocity.getVector());
   }

   public void set(SE3TrajectoryPointInterface<?> se3TrajectoryPoint)
   {
      simpleWaypoint.set(se3TrajectoryPoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3d position, Quat4d orientation, Vector3d linearVelocity,
         Vector3d angularVelocity)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public void setIncludingFrame(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      position.checkReferenceFrameMatch(orientation);
      position.checkReferenceFrameMatch(linearVelocity);
      position.checkReferenceFrameMatch(angularVelocity);
      setToZero(position.getReferenceFrame());
      simpleWaypoint.set(time, position.getPoint(), orientation.getQuaternion(), linearVelocity.getVector(), angularVelocity.getVector());
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SE3TrajectoryPointInterface<?> se3TrajectoryPoint)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(se3TrajectoryPoint);
   }

   @Override
   public void setPositionToZero()
   {
      simpleWaypoint.setPositionToZero();
   }

   @Override
   public void setOrientationToZero()
   {
      simpleWaypoint.setOrientationToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      simpleWaypoint.setLinearVelocityToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      simpleWaypoint.setAngularVelocityToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      simpleWaypoint.setPositionToNaN();
   }

   @Override
   public void setOrientationToNaN()
   {
      simpleWaypoint.setOrientationToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      simpleWaypoint.setLinearVelocityToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      simpleWaypoint.setAngularVelocityToNaN();
   }

   @Override
   public double positionDistance(FrameSE3TrajectoryPoint other)
   {
      checkReferenceFrameMatch(other);
      return simpleWaypoint.positionDistance(other.simpleWaypoint);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      simpleWaypoint.getPosition(positionToPack);
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      simpleWaypoint.getOrientation(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      simpleWaypoint.getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      simpleWaypoint.getAngularVelocity(angularVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      simpleWaypoint.getPosition(positionToPack.getPoint());
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      simpleWaypoint.getOrientation(orientationToPack.getQuaternion());
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      simpleWaypoint.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      simpleWaypoint.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      positionToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getPosition(positionToPack.getPoint());
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getOrientation(orientationToPack.getQuaternion());
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getAngularVelocity(angularVelocityToPack.getVector());
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());
      return "SE3 trajectory point: (" + timeToString + ", " + simpleWaypoint + ")";
   }
}
