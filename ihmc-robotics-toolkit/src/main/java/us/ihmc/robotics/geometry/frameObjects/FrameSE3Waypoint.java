package us.ihmc.robotics.geometry.frameObjects;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.geometry.transformables.SE3Waypoint;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;

public class FrameSE3Waypoint extends FrameGeometryObject<FrameSE3Waypoint, SE3Waypoint> implements SE3WaypointInterface<FrameSE3Waypoint>
{
   private final SE3Waypoint geometryObject;
   
   public FrameSE3Waypoint()
   {
      super(new SE3Waypoint());
      geometryObject = getGeometryObject();
   }

   public FrameSE3Waypoint(ReferenceFrame referenceFrame)
   {
      this();
      this.referenceFrame = referenceFrame;
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      geometryObject.setPosition(position);
   }

   public void setPosition(FramePoint3D position)
   {
      checkReferenceFrameMatch(position);
      geometryObject.setPosition(position);
   }

   @Override
   public void setOrientation(QuaternionReadOnly orientation)
   {
      geometryObject.setOrientation(orientation);
   }

   public void setOrientation(FrameQuaternion orientation)
   {
      geometryObject.setOrientation(orientation);
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      geometryObject.setLinearVelocity(linearVelocity);
   }

   public void setLinearVelocity(FrameVector3D linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      geometryObject.setLinearVelocity(linearVelocity);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      geometryObject.setAngularVelocity(angularVelocity);
   }

   public void setAngularVelocity(FrameVector3D angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.setAngularVelocity(angularVelocity);
   }

   public void set(Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      geometryObject.set(position, orientation, linearVelocity, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Quaternion orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setToZero(referenceFrame);
      geometryObject.set(position, orientation, linearVelocity, angularVelocity);
   }

   public void set(FramePoint3D position, FrameQuaternion orientation, FrameVector3D linearVelocity, FrameVector3D angularVelocity)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(linearVelocity);
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.set(position, orientation, linearVelocity, angularVelocity);
   }

   public void setIncludingFrame(FramePoint3D position, FrameQuaternion orientation, FrameVector3D linearVelocity, FrameVector3D angularVelocity)
   {
      position.checkReferenceFrameMatch(orientation);
      position.checkReferenceFrameMatch(linearVelocity);
      position.checkReferenceFrameMatch(angularVelocity);
      setToZero(position.getReferenceFrame());
      geometryObject.set(position, orientation, linearVelocity, angularVelocity);
   }

   public void set(SE3WaypointInterface<?> se3Waypoint)
   {
      geometryObject.set(se3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SE3WaypointInterface<?> se3Waypoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(se3Waypoint);
   }

   public void set(EuclideanWaypointInterface<?> euclideanWaypoint, SO3WaypointInterface<?> so3Waypoint)
   {
      geometryObject.set(euclideanWaypoint, so3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanWaypointInterface<?> euclideanWaypoint, SO3WaypointInterface<?> so3Waypoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(euclideanWaypoint, so3Waypoint);
   }

   public void set(FrameEuclideanWaypoint frameEuclideanWaypoint, FrameSO3Waypoint frameSO3Waypoint)
   {
      checkReferenceFrameMatch(frameEuclideanWaypoint);
      checkReferenceFrameMatch(frameSO3Waypoint);
      geometryObject.set(frameEuclideanWaypoint.getGeometryObject(), frameSO3Waypoint.getGeometryObject());
   }

   public void setIncludingFrame(FrameEuclideanWaypoint frameEuclideanWaypoint, FrameSO3Waypoint frameSO3Waypoint)
   {
      frameEuclideanWaypoint.checkReferenceFrameMatch(frameSO3Waypoint);
      setToZero(frameEuclideanWaypoint.getReferenceFrame());
      geometryObject.set(frameEuclideanWaypoint.getGeometryObject(), frameSO3Waypoint.getGeometryObject());
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
   public double positionDistance(FrameSE3Waypoint other)
   {
      checkReferenceFrameMatch(other);
      return geometryObject.positionDistance(other.geometryObject);
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      geometryObject.getPosition(positionToPack);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      geometryObject.getPosition(positionToPack);
   }

   public void getPositionIncludingFrame(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(getReferenceFrame());
      geometryObject.getPosition(positionToPack);
   }

   @Override
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      geometryObject.getOrientation(orientationToPack);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      geometryObject.getOrientation(orientationToPack);
   }

   public void getOrientationIncludingFrame(FrameQuaternion orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      geometryObject.getOrientation(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      geometryObject.getLinearVelocity(linearVelocityToPack);
   }

   public void getLinearVelocity(FrameVector3D linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      geometryObject.getLinearVelocity(linearVelocityToPack);
   }

   public void getLinearVelocityIncludingFrame(FrameVector3D linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(getReferenceFrame());
      geometryObject.getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularVelocityIncludingFrame(FrameVector3D angularVelocityToPack)
   {
      setToZero(getReferenceFrame());
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void get(SE3WaypointInterface<?> se3Waypoint)
   {
      EuclideanWaypoint euclideanWaypoint = geometryObject.getEuclideanWaypoint();
      SO3Waypoint so3Waypoint = geometryObject.getSO3Waypoint();

      se3Waypoint.setPosition(euclideanWaypoint.getPosition());
      se3Waypoint.setLinearVelocity(euclideanWaypoint.getLinearVelocity());
      
      se3Waypoint.setOrientation(so3Waypoint.getOrientation());
      se3Waypoint.setAngularVelocity(so3Waypoint.getAngularVelocity());
   }
}
