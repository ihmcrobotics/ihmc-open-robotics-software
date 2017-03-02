package us.ihmc.robotics.geometry.yoFrameObjects;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.geometry.frameObjects.FrameSO3Waypoint;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSO3Waypoint extends YoFrameWaypoint<YoFrameSO3Waypoint, FrameSO3Waypoint, SO3Waypoint>
      implements SO3WaypointInterface<YoFrameSO3Waypoint>
{
   private final YoFrameQuaternion orientation;
   private final YoFrameVector angularVelocity;

   public YoFrameSO3Waypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(new FrameSO3Waypoint(), namePrefix, nameSuffix, registry, referenceFrames);

      orientation = createYoOrientation(this, namePrefix, nameSuffix, registry);
      angularVelocity = createYoAngularVelocity(this, namePrefix, nameSuffix, registry);
   }

   public static YoFrameQuaternion createYoOrientation(final ReferenceFrameHolder referenceFrameHolder, String namePrefix, String nameSuffix,
         YoVariableRegistry registry)
   {
      return new YoFrameQuaternion(createName(namePrefix, "orientation", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }
      };
   }

   public static YoFrameVector createYoAngularVelocity(final ReferenceFrameHolder referenceFrameHolder, String namePrefix, String nameSuffix,
         YoVariableRegistry registry)
   {
      return new YoFrameVector(createName(namePrefix, "angularVelocity", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }
      };
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
   public void setOrientationToZero()
   {
      orientation.setToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      angularVelocity.setToZero();
   }

   @Override
   public void setOrientationToNaN()
   {
      orientation.setToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      angularVelocity.setToNaN();
   }

   @Override
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      orientation.get(orientationToPack);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   public void set(SO3WaypointInterface<?> so3Waypoint)
   {
      frameWaypoint.set(frameWaypoint);
      getYoValuesFromFrameWaypoint();
   }

   @Override
   protected void putYoValuesIntoFrameWaypoint()
   {
      SO3Waypoint simpleWaypoint = frameWaypoint.getGeometryObject();
      simpleWaypoint.set(orientation.getFrameOrientation().getQuaternion(), angularVelocity.getFrameTuple().getVector());
   }

   @Override
   protected void getYoValuesFromFrameWaypoint()
   {
      SO3Waypoint simpleWaypoint = frameWaypoint.getGeometryObject();
      orientation.set(simpleWaypoint.getOrientation());
      angularVelocity.set(simpleWaypoint.getAngularVelocity());
   }
}
