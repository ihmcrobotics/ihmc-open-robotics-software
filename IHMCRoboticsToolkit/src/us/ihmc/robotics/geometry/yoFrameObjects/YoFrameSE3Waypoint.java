package us.ihmc.robotics.geometry.yoFrameObjects;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.frameObjects.FrameSE3Waypoint;
import us.ihmc.robotics.geometry.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.geometry.transformables.SE3Waypoint;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSE3Waypoint extends YoFrameWaypoint<YoFrameSE3Waypoint, FrameSE3Waypoint, SE3Waypoint>
      implements SE3WaypointInterface<YoFrameSE3Waypoint>
{
   private final YoFramePoint position;
   private final YoFrameQuaternion orientation;
   private final YoFrameVector linearVelocity;
   private final YoFrameVector angularVelocity;

   public YoFrameSE3Waypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(new FrameSE3Waypoint(), namePrefix, nameSuffix, registry, referenceFrames);
      position = YoFrameEuclideanWaypoint.createYoPosition(this, namePrefix, nameSuffix, registry);
      orientation = YoFrameSO3Waypoint.createYoOrientation(this, namePrefix, nameSuffix, registry);
      linearVelocity = YoFrameEuclideanWaypoint.createYoLinearVelocity(this, namePrefix, nameSuffix, registry);
      angularVelocity = YoFrameSO3Waypoint.createYoAngularVelocity(this, namePrefix, nameSuffix, registry);
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      this.position.set(position);
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   @Override
   public void setOrientation(QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   @Override
   public void setPositionToZero()
   {
      position.setToZero();
   }

   @Override
   public void setOrientationToZero()
   {
      orientation.setToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      linearVelocity.setToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      angularVelocity.setToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      position.setToNaN();
   }

   @Override
   public void setOrientationToNaN()
   {
      orientation.setToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      linearVelocity.setToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      angularVelocity.setToNaN();
   }

   @Override
   public double positionDistance(YoFrameSE3Waypoint other)
   {
      putYoValuesIntoFrameWaypoint();
      other.putYoValuesIntoFrameWaypoint();
      return frameWaypoint.positionDistance(other.frameWaypoint);
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      position.get(positionToPack);
   }

   @Override
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      orientation.get(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocity.get(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   @Override
   protected void putYoValuesIntoFrameWaypoint()
   {
      SE3Waypoint simpleWaypoint = frameWaypoint.getGeometryObject();
      EuclideanWaypoint euclideanWaypoint = simpleWaypoint.getEuclideanWaypoint();
      SO3Waypoint so3Waypoint = simpleWaypoint.getSO3Waypoint();

      euclideanWaypoint.set(position.getFrameTuple().getPoint(), linearVelocity.getFrameTuple().getVector());
      so3Waypoint.set(orientation.getFrameOrientation().getQuaternion(), angularVelocity.getFrameTuple().getVector());
   }

   @Override
   protected void getYoValuesFromFrameWaypoint()
   {
      SE3Waypoint simpleWaypoint = frameWaypoint.getGeometryObject();
      EuclideanWaypoint euclideanWaypoint = simpleWaypoint.getEuclideanWaypoint();
      SO3Waypoint so3Waypoint = simpleWaypoint.getSO3Waypoint();

      position.set(euclideanWaypoint.getPosition());
      orientation.set(so3Waypoint.getOrientation());
      linearVelocity.set(euclideanWaypoint.getLinearVelocity());
      angularVelocity.set(so3Waypoint.getAngularVelocity());
   }
}
