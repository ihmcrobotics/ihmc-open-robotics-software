package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.frameObjects.FrameSO3Waypoint;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSO3TrajectoryPoint extends FrameTrajectoryPoint<FrameSO3TrajectoryPoint, SimpleSO3TrajectoryPoint>
      implements SO3TrajectoryPointInterface<FrameSO3TrajectoryPoint>
{
   private final SimpleSO3TrajectoryPoint geometryObject;
   
   public FrameSO3TrajectoryPoint()
   {
      super(new SimpleSO3TrajectoryPoint());
      geometryObject = getGeometryObject();
   }

   public FrameSO3TrajectoryPoint(ReferenceFrame referenceFrame)
   {
      this();
      setToZero(referenceFrame);
   }

   public FrameSO3TrajectoryPoint(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      this();
      setIncludingFrame(time, orientation, angularVelocity);
   }

   public FrameSO3TrajectoryPoint(ReferenceFrame referenceFrame, SO3TrajectoryPointInterface<?> so3TrajectoryPointInterface)
   {
      this();
      setIncludingFrame(referenceFrame, so3TrajectoryPointInterface);
   }

   public FrameSO3TrajectoryPoint(FrameSO3TrajectoryPoint frameSO3TrajectoryPoint)
   {
      this();
      setIncludingFrame(frameSO3TrajectoryPoint);
   }

   @Override
   public void setOrientation(QuaternionReadOnly orientation)
   {
      geometryObject.setOrientation(orientation);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      checkReferenceFrameMatch(orientation);
      geometryObject.setOrientation(orientation.getQuaternion());
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      geometryObject.setAngularVelocity(angularVelocity);
   }

   public void setAngularVelocity(FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.setAngularVelocity(angularVelocity.getVector());
   }

   public void set(double time, Quaternion orientation, Vector3D angularVelocity)
   {
      geometryObject.set(time, orientation, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setToZero(referenceFrame);
      geometryObject.set(time, orientation, angularVelocity);
   }

   public void set(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.set(time, orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void setIncludingFrame(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      orientation.checkReferenceFrameMatch(angularVelocity);
      setToZero(orientation.getReferenceFrame());
      geometryObject.set(time, orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void set(double time, SO3WaypointInterface<?> so3Waypoint)
   {
      geometryObject.set(time, so3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, SO3WaypointInterface<?> so3Waypoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(time, so3Waypoint);
   }

   public void set(SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      geometryObject.set(so3TrajectoryPoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(so3TrajectoryPoint);
   }

   public void set(double time, FrameSO3Waypoint frameSO3Waypoint)
   {
      checkReferenceFrameMatch(frameSO3Waypoint);
      setTime(time);
      frameSO3Waypoint.get(geometryObject);
   }

   public void setIncludingFrame(double time, FrameSO3Waypoint frameSO3Waypoint)
   {
      setToZero(frameSO3Waypoint.getReferenceFrame());
      setTime(time);
      frameSO3Waypoint.get(geometryObject);
   }

   @Override
   public void setOrientationToZero()
   {
      geometryObject.setOrientationToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      geometryObject.setAngularVelocityToZero();
   }

   @Override
   public void setOrientationToNaN()
   {
      geometryObject.setOrientationToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      geometryObject.setAngularVelocityToNaN();
   }

   public void getSO3Waypoint(SO3Waypoint so3WaypointToPack)
   {
      geometryObject.get(so3WaypointToPack);
   }
   
   public void getFrameSO3Waypoint(FrameSO3Waypoint frameSO3Waypoint)
   {
      checkReferenceFrameMatch(frameSO3Waypoint);
 
      QuaternionReadOnly orientation = geometryObject.getOrientation();
      Vector3DReadOnly angularVelocity = geometryObject.getAngularVelocity();

      frameSO3Waypoint.set(orientation, angularVelocity);
   }

   @Override
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      geometryObject.getOrientation(orientationToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      geometryObject.getOrientation(orientationToPack.getQuaternion());
   }

   public FrameOrientation getOrientationCopy()
   {
      FrameOrientation orientationCopy = new FrameOrientation(getReferenceFrame());
      getOrientation(orientationCopy);
      return orientationCopy;
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      geometryObject.getOrientation(orientationToPack.getQuaternion());
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      geometryObject.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public FrameVector getAngularVelocityCopy()
   {
      FrameVector angularVelocityCopy = new FrameVector(getReferenceFrame());
      getAngularVelocity(angularVelocityCopy);
      return angularVelocityCopy;
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(getReferenceFrame());
      geometryObject.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public double get(QuaternionBasics orientationToPack, Vector3DBasics angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      return getTime();
   }

   public double get(FrameOrientation orientationToPack, FrameVector angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      return getTime();
   }

   public double getIncludingFrame(FrameOrientation orientationToPack, FrameVector angularVelocityToPack)
   {
      getOrientationIncludingFrame(orientationToPack);
      getAngularVelocityIncludingFrame(angularVelocityToPack);
      return getTime();
   }

   public double getOrientationQx()
   {
      return geometryObject.getOrientationQx();
   }

   public double getOrientationQy()
   {
      return geometryObject.getOrientationQy();
   }

   public double getOrientationQz()
   {
      return geometryObject.getOrientationQz();
   }

   public double getOrientationQs()
   {
      return geometryObject.getOrientationQs();
   }

   public double getAngularVelocityX()
   {
      return geometryObject.getAngularVelocityX();
   }

   public double getAngularVelocityY()
   {
      return geometryObject.getAngularVelocityY();
   }

   public double getAngularVelocityZ()
   {
      return geometryObject.getAngularVelocityZ();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());
      return "SO3 trajectory point: (" + timeToString + ", " + geometryObject + getReferenceFrame() + ")";
   }
}
