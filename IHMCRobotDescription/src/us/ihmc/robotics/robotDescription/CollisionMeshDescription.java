package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;

public class CollisionMeshDescription
{
   private final RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
   private final ArrayList<ConvexShapeDescription> collisionObjects = new ArrayList<>();

   public void addConvexShape(ConvexShapeDescription convexShapeDescription)
   {
      collisionObjects.add(convexShapeDescription);
   }

   public void addSphere(double radius)
   {
      SphereDescriptionReadOnly sphere = new SphereDescriptionReadOnly(radius, transformGenerator.getRigidBodyTransformCopy());
      collisionObjects.add(sphere);
   }

   public void addCubeReferencedAtBottomCenter(double lengthX, double widthY, double heightZ)
   {
      transformGenerator.translate(0.0, 0.0, heightZ / 2.0);
      CubeDescriptionReadOnly cube = new CubeDescriptionReadOnly(lengthX, widthY, heightZ, transformGenerator.getRigidBodyTransformCopy());
      transformGenerator.translate(0.0, 0.0, -heightZ / 2.0);
      collisionObjects.add(cube);
   }

   public void addCubeReferencedAtCenter(double lengthX, double widthY, double heightZ)
   {
      CubeDescriptionReadOnly cube = new CubeDescriptionReadOnly(lengthX, widthY, heightZ, transformGenerator.getRigidBodyTransformCopy());
      collisionObjects.add(cube);
   }

   public void addCapsule(double radius, double height)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, height, transformGenerator.getRigidBodyTransformCopy());
      collisionObjects.add(capsule);
   }

   public void addCapsule(double radius, LineSegment3d capToCapLineSegment)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, capToCapLineSegment, transformGenerator.getRigidBodyTransformCopy());
      collisionObjects.add(capsule);
   }

   public void addCapsule(double radius, double height, Axis longAxis)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, height, longAxis, transformGenerator.getRigidBodyTransformCopy());
      collisionObjects.add(capsule);
   }

   public void addConvexPolytope(ConvexPolytope polytope)
   {
      ConvexPolytopeDescriptionReadOnly polytopeReadOnly = new ConvexPolytopeDescriptionReadOnly(polytope, transformGenerator.getRigidBodyTransformCopy());
      collisionObjects.add(polytopeReadOnly);
   }
}
