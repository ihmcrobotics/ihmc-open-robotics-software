package us.ihmc.simulationconstructionset.physics;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Link;

public interface CollisionShapeWithLink extends CollisionShape
{
   /**
    * The {@link Link} which this shape is attached to.
    */
   public Link getLink();

   /**
    * Transform from shape to link coordinates.
    */
   public void getShapeToLink(RigidBodyTransform shapeToLinkToPack);
}
