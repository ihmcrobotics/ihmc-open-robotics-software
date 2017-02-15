package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotDescription.CapsuleDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.ConvexShapeDescription;
import us.ihmc.robotics.robotDescription.CubeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CylinderDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.SphereDescriptionReadOnly;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.SimpleCollisionShapeWithLink;

public class SimpleCollisionShapeFactory implements CollisionShapeFactory
{
   private final SimpleCollisionDetector detector;
   private double margin;

   public SimpleCollisionShapeFactory(SimpleCollisionDetector detector)
   {
      this.detector = detector;
   }

   @Override
   public void setMargin(double margin)
   {
      this.margin = margin;
   }

   @Override
   public CollisionShapeDescription<?> createBox(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      ConvexPolytope polytope = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
      return new PolytopeShapeDescription(polytope);
//      return new BoxShapeDescription(halfLengthX, halfWidthY, halfHeightZ);
   }

   @Override
   public CollisionShapeDescription<?> createCylinder(double radius, double height)
   {
      return new CylinderShapeDescription(radius, height);
   }

   @Override
   public CollisionShapeDescription<?> createSphere(double radius)
   {
      return new SphereShapeDescription(radius, new Point3d());
   }

   @Override
   public CollisionShapeDescription<?> createCapsule(double radius, double height)
   {
      return new CapsuleShapeDescription(radius, height);
   }

   public CollisionShapeDescription<?> createCapsule(double radius, LineSegment3d capToCapLineSegment)
   {
      return new CapsuleShapeDescription(radius, capToCapLineSegment);
   }

   @Override
   public CollisionShape addShape(Link link, RigidBodyTransform shapeToLink, CollisionShapeDescription<?> description, boolean isGround, int groupMask, int collisionMask)
   {
      SimpleCollisionShapeWithLink collisionShape = new SimpleCollisionShapeWithLink(link, description, shapeToLink);
      collisionShape.setIsGround(isGround);
      collisionShape.setCollisionGroup(groupMask);
      collisionShape.setCollisionMask(collisionMask);
      detector.addShape(collisionShape);

      return collisionShape;
   }

   @Override
   public CollisionShape addShape(CollisionShapeDescription<?> description)
   {
      Link link = null;
      RigidBodyTransform shapeToLink = new RigidBodyTransform();
      boolean isGround = false;
      int collisionGroup = 0xFFFF;
      int collisionMask = 0xFFFF;

      return addShape(link, shapeToLink, description, isGround, collisionGroup, collisionMask);
   }

   @Override
   public void addCollisionMeshDescription(Link link, CollisionMeshDescription collisionMeshDescription)
   {
      ArrayList<ConvexShapeDescription> convexShapeDescriptions = new ArrayList<>();
      collisionMeshDescription.getConvexShapeDescriptions(convexShapeDescriptions);

      for (ConvexShapeDescription convexShapeDescription : convexShapeDescriptions)
      {
         if (convexShapeDescription instanceof CapsuleDescriptionReadOnly)
         {
            CapsuleDescriptionReadOnly capsule = (CapsuleDescriptionReadOnly) convexShapeDescription;

            LineSegment3d capToCapLineSegment = new LineSegment3d();
            capsule.getCapToCapLineSegment(capToCapLineSegment);

            CollisionShapeDescription<?> collisionShapeDescription = createCapsule(capsule.getRadius(), capToCapLineSegment);
            addShape(link, null, collisionShapeDescription, collisionMeshDescription.getIsGround(), collisionMeshDescription.getCollisionGroup(), collisionMeshDescription.getCollisionMask());
         }

         else if (convexShapeDescription instanceof SphereDescriptionReadOnly)
         {
            SphereDescriptionReadOnly sphere = (SphereDescriptionReadOnly) convexShapeDescription;
            CollisionShapeDescription<?> collisionShapeDescription = createSphere(sphere.getRadius());
            RigidBodyTransform transform = new RigidBodyTransform();
            sphere.getRigidBodyTransform(transform);
            addShape(link, transform, collisionShapeDescription, collisionMeshDescription.getIsGround(), collisionMeshDescription.getCollisionGroup(), collisionMeshDescription.getCollisionMask());
         }

         else if (convexShapeDescription instanceof CubeDescriptionReadOnly)
         {
            CubeDescriptionReadOnly cube = (CubeDescriptionReadOnly) convexShapeDescription;
            CollisionShapeDescription<?> collisionShapeDescription = createBox(cube.getLengthX()/2.0, cube.getWidthY()/2.0, cube.getHeightZ()/2.0);
            RigidBodyTransform transform = new RigidBodyTransform();
            cube.getRigidBodyTransformToCenter(transform);
            addShape(link, transform, collisionShapeDescription, collisionMeshDescription.getIsGround(), collisionMeshDescription.getCollisionGroup(), collisionMeshDescription.getCollisionMask());
         }

         else if (convexShapeDescription instanceof CylinderDescriptionReadOnly)
         {
            CylinderDescriptionReadOnly cylinder = (CylinderDescriptionReadOnly) convexShapeDescription;
            CollisionShapeDescription<?> collisionShapeDescription = createCylinder(cylinder.getRadius(), cylinder.getHeight());
            RigidBodyTransform transform = new RigidBodyTransform();
            cylinder.getRigidBodyTransformToCenter(transform);
            addShape(link, transform, collisionShapeDescription, collisionMeshDescription.getIsGround(), collisionMeshDescription.getCollisionGroup(), collisionMeshDescription.getCollisionMask());
         }

         else
         {
            throw new RuntimeException(getClass().getSimpleName() + ". Don't recognize convexShapeDescription type!");
         }
      }

   }

}
