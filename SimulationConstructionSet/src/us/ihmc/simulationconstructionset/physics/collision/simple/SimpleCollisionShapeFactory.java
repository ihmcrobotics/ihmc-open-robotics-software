package us.ihmc.simulationconstructionset.physics.collision.simple;

import javax.vecmath.Point3d;

import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.robotics.geometry.RigidBodyTransform;
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
   public CollisionShapeDescription createBox(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      ConvexPolytope polytope = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
      return new PolytopeShapeDescription(polytope);
//      return new BoxShapeDescription(halfLengthX, halfWidthY, halfHeightZ);
   }

   @Override
   public CollisionShapeDescription createCylinder(double radius, double height)
   {
      return new CylinderShapeDescription(radius, height);
   }

   @Override
   public CollisionShapeDescription createSphere(double radius)
   {
      return new SphereShapeDescription(radius, new Point3d());
   }
   
   @Override
   public CollisionShapeDescription createCapsule(double radius, double height)
   {
      return new CapsuleShapeDescription(radius, height);
   }

   @Override
   public CollisionShape addShape(Link link, RigidBodyTransform shapeToLink, CollisionShapeDescription description, boolean isGround, int collisionGroup, int collisionMask)
   {
      SimpleCollisionShapeWithLink collisionShape = new SimpleCollisionShapeWithLink(link, description, shapeToLink);

      detector.addShape(collisionShape);

      return collisionShape;
   }

   @Override
   public CollisionShape addShape(CollisionShapeDescription description)
   {
      Link link = null;
      RigidBodyTransform shapeToLink = new RigidBodyTransform();
      boolean isGround = false;
      int collisionGroup = 0xFFFF;
      int collisionMask = 0xFFFF;

      return addShape(link, shapeToLink, description, isGround, collisionGroup, collisionMask);
   }

}
