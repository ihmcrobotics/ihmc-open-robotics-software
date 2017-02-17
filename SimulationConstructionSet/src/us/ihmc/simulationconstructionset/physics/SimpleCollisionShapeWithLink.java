package us.ihmc.simulationconstructionset.physics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShape;

public class SimpleCollisionShapeWithLink extends SimpleCollisionShape implements CollisionShapeWithLink
{
   private final Link link;

   private final RigidBodyTransform shapeToLink = new RigidBodyTransform();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public SimpleCollisionShapeWithLink(Link link, CollisionShapeDescription collisionShapeDescription, RigidBodyTransform shapeToLink)
   {
      super(collisionShapeDescription);
      this.link = link;

      if (shapeToLink != null)
      {
         this.shapeToLink.set(shapeToLink);
      }
   }

   @Override
   public Link getLink()
   {
      return link;
   }

   @Override
   public void getShapeToLink(RigidBodyTransform shapeToLinkToPack)
   {
      shapeToLinkToPack.set(shapeToLink);
   }

   @Override
   public void getTransformToWorld(RigidBodyTransform transformToWorldToPack)
   {
      if (link != null)
      {
         link.getParentJoint().getTransformToWorld(tempTransform);
         transformToWorldToPack.set(tempTransform);
         transformToWorldToPack.multiply(shapeToLink);
      }
      else
      {
         super.getTransformToWorld(transformToWorldToPack);
      }
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      if (link != null)
      {
         throw new RuntimeException("Shouldn't call this!");
      }
      else
      {
         super.setTransformToWorld(transformToWorld);
      }
   }

}
