package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;

public class SimpleCollisionShapeFactory implements CollisionShapeFactory
{
   private final SimpleCollisionDetector detector;

   public SimpleCollisionShapeFactory(SimpleCollisionDetector detector)
   {
      this.detector = detector;
   }

   @Override
   public void setMargin(double margin)
   {
   }

   @Override
   public CollisionShapeDescription createBox(double radiusX, double radiusY, double radiusZ)
   {
      return null;
   }

   @Override
   public CollisionShapeDescription createCylinder(double radius, double height)
   {
      return null;
   }

   @Override
   public CollisionShapeDescription createSphere(double radius)
   {
      return new SphereShapeDescription(radius);
   }

   @Override
   public CollisionShape addShape(Link link, RigidBodyTransform shapeToLink, CollisionShapeDescription description, boolean isGround, int collisionGroup, int collisionMask)
   {
      CollisionObject collisionShape = new CollisionObject(description);

      detector.addShape(collisionShape);

      return collisionShape;
   }

}
