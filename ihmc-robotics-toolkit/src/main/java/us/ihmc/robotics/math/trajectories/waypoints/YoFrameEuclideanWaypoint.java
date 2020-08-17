package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFrameEuclideanWaypoint implements FrameEuclideanWaypointBasics
{
   private final YoMutableFramePoint3D position;
   private final YoMutableFrameVector3D linearVelocity;

   public YoFrameEuclideanWaypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      position = new YoMutableFramePoint3D(namePrefix + "Position", nameSuffix, registry);
      linearVelocity = new YoMutableFrameVector3D(namePrefix + "LinearVelocity", nameSuffix, registry);
   }

   public YoDouble getYoX()
   {
      return position.getYoX();
   }

   public YoDouble getYoY()
   {
      return position.getYoY();
   }

   public YoDouble getYoZ()
   {
      return position.getYoZ();
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return position;
   }

   @Override
   public FrameVector3DReadOnly getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   @Override
   public void setLinearVelocity(double x, double y, double z)
   {
      linearVelocity.set(x, y, z);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      position.checkReferenceFrameMatch(linearVelocity);
      return position.getReferenceFrame();
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      position.setReferenceFrame(referenceFrame);
      linearVelocity.setReferenceFrame(referenceFrame);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      position.applyTransform(transform);
      linearVelocity.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      position.applyInverseTransform(transform);
      linearVelocity.applyInverseTransform(transform);
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
