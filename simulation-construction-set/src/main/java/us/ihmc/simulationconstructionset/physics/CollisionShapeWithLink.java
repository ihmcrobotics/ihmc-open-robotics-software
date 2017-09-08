package us.ihmc.simulationconstructionset.physics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Link;

public interface CollisionShapeWithLink extends CollisionShape
{
   /**
    * The {@link Link} which this shape is attached to.
    */
   public abstract Link getLink();

   /**
    * Transform from shape to link coordinates.
    */
   public abstract void getShapeToLink(RigidBodyTransform shapeToLinkToPack);

}
