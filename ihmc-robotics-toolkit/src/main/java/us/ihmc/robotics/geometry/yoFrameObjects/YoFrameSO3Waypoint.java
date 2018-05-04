package us.ihmc.robotics.geometry.yoFrameObjects;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.geometry.frameObjects.FrameSO3Waypoint;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;

public class YoFrameSO3Waypoint extends YoFrameWaypoint<YoFrameSO3Waypoint, FrameSO3Waypoint, SO3Waypoint>
      implements SO3WaypointInterface<YoFrameSO3Waypoint>
{
   private final YoFrameQuaternion orientation;
   private final YoFrameVector3D angularVelocity;

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

   public static YoFrameVector3D createYoAngularVelocity(final ReferenceFrameHolder referenceFrameHolder, String namePrefix, String nameSuffix,
         YoVariableRegistry registry)
   {
      return new YoFrameVector3D(createName(namePrefix, "angularVelocity", ""), nameSuffix, null, registry)
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
      orientationToPack.set(orientation);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
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
      simpleWaypoint.set(orientation, angularVelocity);
   }

   @Override
   protected void getYoValuesFromFrameWaypoint()
   {
      SO3Waypoint simpleWaypoint = frameWaypoint.getGeometryObject();
      orientation.set(simpleWaypoint.getOrientation());
      angularVelocity.set(simpleWaypoint.getAngularVelocity());
   }
}
