package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

@Deprecated // Not finished
public class RampRayIntersection
{
   private final RigidBodyTransform rampToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame rampFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                   rampToWorldTransform);
   private final FrameRamp3D ramp = new FrameRamp3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private boolean intersects = false;

   public void update(Vector3DReadOnly size, Point3DReadOnly position, UnitVector3DReadOnly axis, ReferenceFrame referenceFrame)
   {
      referenceFrame.getTransformToDesiredFrame(rampToWorldTransform, ReferenceFrame.getWorldFrame());
      rampFrame.update();
      ramp.setToZero(rampFrame);
      ramp.getSize().set(size);
      ramp.getPosition().set(position);
//      ramp.getOrientation().set(axis);
   }

   public void update(double length, double radius, double zOffset, RigidBodyTransformReadOnly transformToWorld)
   {
      rampToWorldTransform.set(transformToWorld);
      rampFrame.update();
      ramp.setToZero(rampFrame);
//      ramp.setSize(length, radius);
      ramp.getPosition().addZ(zOffset);
   }

   public double intersect(Line3DReadOnly pickRay)
   {
      ramp.changeFrame(ReferenceFrame.getWorldFrame());
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBox3D(ramp.getPosition(),
                                                                                       ramp.getOrientation(),
                                                                                       ramp.getSize(),
                                                                                       pickRay.getPoint(),
                                                                                       pickRay.getDirection(),
                                                                                       firstIntersectionToPack,
                                                                                       secondIntersectionToPack);
      intersects = numberOfIntersections == 2;
      return intersects ? firstIntersectionToPack.distance(pickRay.getPoint()) : Double.NaN;
   }

   public Point3DReadOnly getClosestIntersection()
   {
      return firstIntersectionToPack;
   }

   public Ramp3DReadOnly getRamp()
   {
      return ramp;
   }

   public boolean getIntersects()
   {
      return intersects;
   }
}
