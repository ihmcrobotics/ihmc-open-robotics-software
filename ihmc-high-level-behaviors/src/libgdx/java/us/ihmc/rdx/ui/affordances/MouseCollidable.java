package us.ihmc.rdx.ui.affordances;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.gizmo.*;
import us.ihmc.robotics.physics.Collidable;

public class MouseCollidable
{
   private final FrameShape3DBasics shape;
   private SphereRayIntersection sphereRayIntersection;
   private CapsuleRayIntersection capsuleIntersection;
   private CylinderRayIntersection cylinderRayIntersection;
   private EllipsoidRayIntersection ellipsoidRayIntersection;
   private BoxRayIntersection boxRayIntersection;

   public MouseCollidable(Collidable collidable)
   {
      this((FrameShape3DBasics) collidable.getShape());
   }

   public MouseCollidable(FrameShape3DBasics shape)
   {
      this.shape = shape;

      if (shape instanceof FrameSphere3DReadOnly sphere)
      {
         sphereRayIntersection = new SphereRayIntersection();
      }
      else if (shape instanceof FrameCapsule3DReadOnly capsule)
      {
         capsuleIntersection = new CapsuleRayIntersection();
      }
      else if (shape instanceof FrameBox3DReadOnly box)
      {
         boxRayIntersection = new BoxRayIntersection();
      }
      else if (shape instanceof FramePointShape3DReadOnly pointShape)
      {
         // We're not colliding with points as they have no volume
      }
      else if (shape instanceof FrameCylinder3DReadOnly cylinder)
      {
         cylinderRayIntersection = new CylinderRayIntersection();
      }
      else if (shape instanceof FrameEllipsoid3DReadOnly ellipsoid)
      {
         ellipsoidRayIntersection = new EllipsoidRayIntersection();
      }
      else
      {
         LogTools.warn("Shape not handled: {}", shape);
      }
   }

   public double collide(Line3DReadOnly pickRayInWorld, ReferenceFrame collisionShapeFrame)
   {
      if (shape instanceof Sphere3DReadOnly sphere)
      {
         sphereRayIntersection.update(sphere.getRadius(), sphere.getPosition(), collisionShapeFrame);
         if (sphereRayIntersection.intersect(pickRayInWorld))
         {
            return pickRayInWorld.getPoint().distance(sphereRayIntersection.getFirstIntersectionToPack());
         }
      }
      else if (shape instanceof Capsule3DReadOnly capsule)
      {
         UnitVector3DReadOnly axis = capsule.getAxis();
         Point3DReadOnly position = capsule.getPosition();
         double length = capsule.getLength();
         double radius = capsule.getRadius();
         capsuleIntersection.update(radius, length, position, axis, collisionShapeFrame.getParent());
         if (capsuleIntersection.intersect(pickRayInWorld))
         {
            return capsuleIntersection.getDistanceToCollision(pickRayInWorld);
         }
      }
      else if (shape instanceof Box3DReadOnly box)
      {
         if (boxRayIntersection.intersect(box.getSizeX(),
                                          box.getSizeY(),
                                          box.getSizeZ(),
                                          collisionShapeFrame.getTransformToRoot(),
                                          pickRayInWorld))
         {
            return boxRayIntersection.getFirstIntersectionToPack().distance(pickRayInWorld.getPoint());
         }
      }
      else if (shape instanceof PointShape3DReadOnly pointShape)
      {
         // We're not colliding with points as they have no volume
      }
      else if (shape instanceof Cylinder3DReadOnly cylinder)
      {
         cylinderRayIntersection.update(cylinder.getLength(),
                                        cylinder.getRadius(),
                                        cylinder.getPosition(),
                                        cylinder.getAxis(),
                                        collisionShapeFrame.getParent());
         return cylinderRayIntersection.intersect(pickRayInWorld);
      }
      else if (shape instanceof Ellipsoid3DReadOnly ellipsoid)
      {
         ellipsoidRayIntersection.update(ellipsoid.getRadiusX(),
                                         ellipsoid.getRadiusY(),
                                         ellipsoid.getRadiusZ(),
                                         ellipsoid.getPosition(),
                                         ellipsoid.getOrientation(),
                                         collisionShapeFrame.getParent());
         if (ellipsoidRayIntersection.intersect(pickRayInWorld))
         {
            return ellipsoidRayIntersection.getFirstIntersectionToPack().distance(pickRayInWorld.getPoint());
         }
      }
      else
      {
         LogTools.warn("Shape not handled: {}", shape);
      }
      return Double.NaN;
   }
}
