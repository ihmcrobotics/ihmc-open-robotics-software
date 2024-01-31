package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class CylinderRayIntersection
{
   private final RigidBodyTransform cylinderToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame cylinderFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                       cylinderToWorldTransform);
   private final FrameCylinder3D cylinder = new FrameCylinder3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private boolean intersects = false;

   public void update(double length, double radius, Point3DReadOnly position, UnitVector3DReadOnly axis, ReferenceFrame referenceFrame)
   {
      referenceFrame.getTransformToDesiredFrame(cylinderToWorldTransform, ReferenceFrame.getWorldFrame());
      cylinderFrame.update();
      cylinder.setToZero(cylinderFrame);
      cylinder.setSize(length, radius);
      cylinder.getPosition().set(position);
      cylinder.getAxis().set(axis);
   }

   public void update(double length, double radius, double zOffset, RigidBodyTransformReadOnly transformToWorld)
   {
      cylinderToWorldTransform.set(transformToWorld);
      cylinderFrame.update();
      cylinder.setToZero(cylinderFrame);
      cylinder.setSize(length, radius);
      cylinder.getPosition().addZ(zOffset);
   }

   public double intersect(Line3DReadOnly pickRay)
   {
      cylinder.changeFrame(ReferenceFrame.getWorldFrame());
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinder.getLength(),
                                                                                            cylinder.getRadius(),
                                                                                            cylinder.getPosition(),
                                                                                            cylinder.getAxis(),
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

   public Cylinder3DReadOnly getCylinder()
   {
      return cylinder;
   }

   public boolean getIntersects()
   {
      return intersects;
   }
}
