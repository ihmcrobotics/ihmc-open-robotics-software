package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.physics.Collidable;

/**
 * Tool for colliding a collision shape with a mouse pick ray.
 * TODO: Rename to RayCollidable.
 */
public class MouseCollidable
{
   private final FrameShape3DReadOnly shape;
   private SphereRayIntersection sphereRayIntersection;
   private CapsuleRayIntersection capsuleIntersection;
   private CylinderRayIntersection cylinderRayIntersection;
   private EllipsoidRayIntersection ellipsoidRayIntersection;
   private BoxRayIntersection boxRayIntersection;

   public MouseCollidable(Collidable collidable)
   {
      this(collidable.getShape());
   }

   public MouseCollidable(FrameShape3DReadOnly shape)
   {
      this.shape = shape;

      if (shape instanceof FrameSphere3DReadOnly)
      {
         sphereRayIntersection = new SphereRayIntersection();
      }
      else if (shape instanceof FrameCapsule3DReadOnly)
      {
         capsuleIntersection = new CapsuleRayIntersection();
      }
      else if (shape instanceof FrameBox3DReadOnly)
      {
         boxRayIntersection = new BoxRayIntersection();
      }
      else if (shape instanceof FrameRamp3DReadOnly)
      {
         // TODO: Implement for ramp
         boxRayIntersection = new BoxRayIntersection();
      }
      else if (shape instanceof FramePointShape3DReadOnly)
      {
         // We're not colliding with points as they have no volume
      }
      else if (shape instanceof FrameCylinder3DReadOnly)
      {
         cylinderRayIntersection = new CylinderRayIntersection();
      }
      else if (shape instanceof FrameEllipsoid3DReadOnly)
      {
         ellipsoidRayIntersection = new EllipsoidRayIntersection();
      }
      else
      {
         LogTools.warn("Shape not handled: {}", shape);
      }
   }

   /**
    * @param pickRayInWorld The mouse's pick ray in world.
    * @param shapePoseFrame The reference frame of the shape's pose. The shape in this class's
    *                       reference frame is before the pose of the shape and it doesn't have
    *                       a reference frame to the actual pose of the shape, which is a more
    *                       convenient frame to do collision math for i.e. a sphere.
    *                       This can change dynamically for some applications
    *                       which is why we don't pass it in in the constructor.
    * @return The closest collision distance of NaN if not colliding
    */
   public double collide(Line3DReadOnly pickRayInWorld, ReferenceFrame shapePoseFrame)
   {
      if (shape instanceof Sphere3DReadOnly sphere)
      {
         sphereRayIntersection.update(sphere.getRadius(), sphere.getPosition(), shapePoseFrame);
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
         capsuleIntersection.update(radius, length, position, axis, shapePoseFrame.getParent());
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
                                          shapePoseFrame.getTransformToRoot(),
                                          pickRayInWorld))
         {
            return boxRayIntersection.getFirstIntersectionToPack().distance(pickRayInWorld.getPoint());
         }
      }
      else if (shape instanceof Ramp3DReadOnly ramp)
      {
         // TODO: Finish implementing RampRayIntersection
         if (boxRayIntersection.intersect(ramp.getSizeX(),
                                          ramp.getSizeY(),
                                          ramp.getSizeZ(),
                                          shapePoseFrame.getTransformToRoot(),
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
                                        shapePoseFrame.getParent());
         return cylinderRayIntersection.intersect(pickRayInWorld);
      }
      else if (shape instanceof Ellipsoid3DReadOnly ellipsoid)
      {
         ellipsoidRayIntersection.update(ellipsoid.getRadiusX(),
                                         ellipsoid.getRadiusY(),
                                         ellipsoid.getRadiusZ(),
                                         ellipsoid.getPosition(),
                                         ellipsoid.getOrientation(),
                                         shapePoseFrame.getParent());
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
