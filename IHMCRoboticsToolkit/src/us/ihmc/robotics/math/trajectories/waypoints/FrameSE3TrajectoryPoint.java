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
   private final SimpleSE3TrajectoryPoint geometryObject;
   
   public FrameSE3TrajectoryPoint()
   {
      super(new SimpleSE3TrajectoryPoint());
      geometryObject = getGeometryObject();
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
      geometryObject.setPosition(position);
   }

   public void setPosition(FramePoint position)
   {
      checkReferenceFrameMatch(position);
      geometryObject.setPosition(position.getPoint());
   }

   @Override
   public void setOrientation(Quat4d orientation)
   {
      geometryObject.setOrientation(orientation);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      checkReferenceFrameMatch(orientation);
      geometryObject.setOrientation(orientation.getQuaternion());
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      geometryObject.setLinearVelocity(linearVelocity);
   }

   public void setLinearVelocity(FrameVector linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      geometryObject.setLinearVelocity(linearVelocity.getVector());
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      geometryObject.setAngularVelocity(angularVelocity);
   }

   public void setAngularVelocity(FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.setAngularVelocity(angularVelocity.getVector());
   }

   public void set(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      geometryObject.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public void set(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(linearVelocity);
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.set(time, position.getPoint(), orientation.getQuaternion(), linearVelocity.getVector(), angularVelocity.getVector());
   }

   public void set(SE3TrajectoryPointInterface<?> se3TrajectoryPoint)
   {
      geometryObject.set(se3TrajectoryPoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3d position, Quat4d orientation, Vector3d linearVelocity,
         Vector3d angularVelocity)
   {
      setToZero(referenceFrame);
      geometryObject.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public void setIncludingFrame(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      position.checkReferenceFrameMatch(orientation);
      position.checkReferenceFrameMatch(linearVelocity);
      position.checkReferenceFrameMatch(angularVelocity);
      setToZero(position.getReferenceFrame());
      geometryObject.set(time, position.getPoint(), orientation.getQuaternion(), linearVelocity.getVector(), angularVelocity.getVector());
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SE3TrajectoryPointInterface<?> se3TrajectoryPoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(se3TrajectoryPoint);
   }

   @Override
   public void setPositionToZero()
   {
      geometryObject.setPositionToZero();
   }

   @Override
   public void setOrientationToZero()
   {
      geometryObject.setOrientationToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      geometryObject.setLinearVelocityToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      geometryObject.setAngularVelocityToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      geometryObject.setPositionToNaN();
   }

   @Override
   public void setOrientationToNaN()
   {
      geometryObject.setOrientationToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      geometryObject.setLinearVelocityToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      geometryObject.setAngularVelocityToNaN();
   }

   @Override
   public double positionDistance(FrameSE3TrajectoryPoint other)
   {
      checkReferenceFrameMatch(other);
      return geometryObject.positionDistance(other.geometryObject);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      geometryObject.getPosition(positionToPack);
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      geometryObject.getOrientation(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      geometryObject.getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      geometryObject.getPosition(positionToPack.getPoint());
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      geometryObject.getOrientation(orientationToPack.getQuaternion());
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      geometryObject.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      geometryObject.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      positionToPack.setToZero(getReferenceFrame());
      geometryObject.getPosition(positionToPack.getPoint());
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      geometryObject.getOrientation(orientationToPack.getQuaternion());
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(getReferenceFrame());
      geometryObject.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(getReferenceFrame());
      geometryObject.getAngularVelocity(angularVelocityToPack.getVector());
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());
      return "SE3 trajectory point: (" + timeToString + ", " + geometryObject + ")";
   }
}
