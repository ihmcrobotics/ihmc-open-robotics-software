package us.ihmc.simulationconstructionset.physics;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
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
