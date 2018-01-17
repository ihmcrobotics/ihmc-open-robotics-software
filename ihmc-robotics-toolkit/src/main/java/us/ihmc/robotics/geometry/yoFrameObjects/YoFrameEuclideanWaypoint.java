package us.ihmc.robotics.geometry.yoFrameObjects;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.frameObjects.FrameEuclideanWaypoint;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFrameEuclideanWaypoint extends YoFrameWaypoint<YoFrameEuclideanWaypoint, FrameEuclideanWaypoint, EuclideanWaypoint>
      implements EuclideanWaypointInterface<YoFrameEuclideanWaypoint>
{
   private final YoFramePoint position;
   private final YoFrameVector linearVelocity;

   public YoFrameEuclideanWaypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(new FrameEuclideanWaypoint(), namePrefix, nameSuffix, registry, referenceFrames);

      position = createYoPosition(this, namePrefix, nameSuffix, registry);
      linearVelocity = createYoLinearVelocity(this, namePrefix, nameSuffix, registry);
   }

   public static YoFramePoint createYoPosition(final ReferenceFrameHolder referenceFrameHolder, String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      return new YoFramePoint(createName(namePrefix, "position", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }
      };
   }

   public static YoFrameVector createYoLinearVelocity(final ReferenceFrameHolder referenceFrameHolder, String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      return new YoFrameVector(createName(namePrefix, "linearVelocity", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }
      };
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
   public void setPositionToZero()
   {
      position.setToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      linearVelocity.setToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      position.setToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      linearVelocity.setToNaN();
   }

   @Override
   public double positionDistance(YoFrameEuclideanWaypoint other)
   {
      return frameWaypoint.positionDistance(other.frameWaypoint);
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      positionToPack.set(position);
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   @Override
   protected void putYoValuesIntoFrameWaypoint()
   {
      EuclideanWaypoint simpleWaypoint = frameWaypoint.getGeometryObject();
      simpleWaypoint.set(position.getFrameTuple().getPoint(), linearVelocity.getFrameTuple().getVector());
   }

   @Override
   protected void getYoValuesFromFrameWaypoint()
   {
      EuclideanWaypoint simpleWaypoint = frameWaypoint.getGeometryObject();
      position.set(simpleWaypoint.getPosition());
      linearVelocity.set(simpleWaypoint.getLinearVelocity());
   }
}
