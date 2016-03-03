package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSE3Waypoint extends YoFrameWaypoint<YoFrameSE3Waypoint, FrameSE3Waypoint, SimpleSE3Waypoint>
      implements SE3WaypointInterface<YoFrameSE3Waypoint>
{
   private final YoFramePoint position;
   private final YoFrameQuaternion orientation;
   private final YoFrameVector linearVelocity;
   private final YoFrameVector angularVelocity;

   protected YoFrameSE3Waypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(new FrameSE3Waypoint(), namePrefix, nameSuffix, registry, referenceFrames);
      position = YoFrameEuclideanWaypoint.createYoPosition(this, namePrefix, nameSuffix, registry);
      orientation = YoFrameSO3Waypoint.createYoOrientation(this, namePrefix, nameSuffix, registry);
      linearVelocity = YoFrameEuclideanWaypoint.createYoLinearVelocity(this, namePrefix, nameSuffix, registry);
      angularVelocity = YoFrameSO3Waypoint.createYoAngularVelocity(this, namePrefix, nameSuffix, registry);
   }

   @Override
   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   @Override
   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
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
   public void getPosition(Point3d positionToPack)
   {
      position.get(positionToPack);
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      orientation.get(orientationToPack);
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

   @Override
   protected void putYoValuesIntoFrameWaypoint()
   {
      SimpleSE3Waypoint simpleWaypoint = frameWaypoint.getSimpleWaypoint();
      SimpleEuclideanWaypoint euclideanWaypoint = simpleWaypoint.getEuclideanWaypoint();
      SimpleSO3Waypoint so3Waypoint = simpleWaypoint.getSO3Waypoint();

      position.get(euclideanWaypoint.getPosition());
      orientation.get(so3Waypoint.getOrientation());
      linearVelocity.get(euclideanWaypoint.getLinearVelocity());
      angularVelocity.get(so3Waypoint.getAngularVelocity());
   }

   @Override
   protected void getYoValuesFromFrameWaypoint()
   {
      SimpleSE3Waypoint simpleWaypoint = frameWaypoint.getSimpleWaypoint();
      SimpleEuclideanWaypoint euclideanWaypoint = simpleWaypoint.getEuclideanWaypoint();
      SimpleSO3Waypoint so3Waypoint = simpleWaypoint.getSO3Waypoint();

      position.set(euclideanWaypoint.getPosition());
      orientation.set(so3Waypoint.getOrientation());
      linearVelocity.set(euclideanWaypoint.getLinearVelocity());
      angularVelocity.set(so3Waypoint.getAngularVelocity());
   }
}
