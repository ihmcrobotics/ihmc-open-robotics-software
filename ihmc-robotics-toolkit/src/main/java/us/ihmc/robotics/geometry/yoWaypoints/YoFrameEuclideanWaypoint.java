package us.ihmc.robotics.geometry.yoWaypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.geometry.interfaces.FrameEuclideanWaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePoint3D;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameVector3D;

public class YoFrameEuclideanWaypoint implements FrameEuclideanWaypointInterface
{
   private final YoMutableFramePoint3D position;
   private final YoMutableFrameVector3D linearVelocity;

   public YoFrameEuclideanWaypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry)
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
