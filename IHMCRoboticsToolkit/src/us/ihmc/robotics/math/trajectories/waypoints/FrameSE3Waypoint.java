package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSE3Waypoint extends FrameWaypoint<FrameSE3Waypoint, SimpleSE3Waypoint> implements SE3WaypointInterface<FrameSE3Waypoint>
{
   public FrameSE3Waypoint()
   {
      super(new SimpleSE3Waypoint());
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

   public void set(Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      simpleWaypoint.set(position, orientation, linearVelocity, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(position, orientation, linearVelocity, angularVelocity);
   }

   public void set(FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(linearVelocity);
      checkReferenceFrameMatch(angularVelocity);
      simpleWaypoint.set(position.getPoint(), orientation.getQuaternion(), linearVelocity.getVector(), angularVelocity.getVector());
   }

   public void setIncludingFrame(FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      position.checkReferenceFrameMatch(orientation);
      position.checkReferenceFrameMatch(linearVelocity);
      position.checkReferenceFrameMatch(angularVelocity);
      setToZero(position.getReferenceFrame());
      simpleWaypoint.set(position.getPoint(), orientation.getQuaternion(), linearVelocity.getVector(), angularVelocity.getVector());
   }

   public void set(SE3WaypointInterface<?> se3Waypoint)
   {
      simpleWaypoint.set(se3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SE3WaypointInterface<?> se3Waypoint)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(se3Waypoint);
   }

   public void set(EuclideanWaypointInterface<?> euclideanWaypoint, SO3WaypointInterface<?> so3Waypoint)
   {
      simpleWaypoint.set(euclideanWaypoint, so3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanWaypointInterface<?> euclideanWaypoint, SO3WaypointInterface<?> so3Waypoint)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(euclideanWaypoint, so3Waypoint);
   }

   public void set(FrameEuclideanWaypoint frameEuclideanWaypoint, FrameSO3Waypoint frameSO3Waypoint)
   {
      checkReferenceFrameMatch(frameEuclideanWaypoint);
      checkReferenceFrameMatch(frameSO3Waypoint);
      simpleWaypoint.set(frameEuclideanWaypoint.simpleWaypoint, frameSO3Waypoint.simpleWaypoint);
   }

   public void setIncludingFrame(FrameEuclideanWaypoint frameEuclideanWaypoint, FrameSO3Waypoint frameSO3Waypoint)
   {
      frameEuclideanWaypoint.checkReferenceFrameMatch(frameSO3Waypoint);
      setToZero(frameEuclideanWaypoint.getReferenceFrame());
      simpleWaypoint.set(frameEuclideanWaypoint.simpleWaypoint, frameSO3Waypoint.simpleWaypoint);
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
   public double positionDistance(FrameSE3Waypoint other)
   {
      checkReferenceFrameMatch(other);
      return simpleWaypoint.positionDistance(other.simpleWaypoint);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      simpleWaypoint.getPosition(positionToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      simpleWaypoint.getPosition(positionToPack.getPoint());
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      positionToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getPosition(positionToPack.getPoint());
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      simpleWaypoint.getOrientation(orientationToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      simpleWaypoint.getOrientation(orientationToPack.getQuaternion());
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getOrientation(orientationToPack.getQuaternion());
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      simpleWaypoint.getLinearVelocity(linearVelocityToPack);
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      simpleWaypoint.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getLinearVelocity(linearVelocityToPack.getVector());
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      simpleWaypoint.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      simpleWaypoint.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      setToZero(getReferenceFrame());
      simpleWaypoint.getAngularVelocity(angularVelocityToPack.getVector());
   }
}
