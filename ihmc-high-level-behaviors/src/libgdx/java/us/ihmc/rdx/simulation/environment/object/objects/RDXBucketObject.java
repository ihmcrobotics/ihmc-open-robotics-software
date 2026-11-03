package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class RDXBucketObject extends RDXEnvironmentObject
{
   public static final String NAME = "Bucket";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXBucketObject.class);

   public RDXBucketObject()
   {
      super(NAME, FACTORY);
      loadRealisticModel("environmentObjects/bucket/bucket.glb");

      setMass(0.5f);
      Box3D collisionBox = new Box3D(0.307895, 0.300675, 0.384);
      collisionBox.getPosition().set(0.003548, -0.000499, 0.192235);
      getBoundingSphere().setRadius(0.288384);
      getBoundingSphere().getPosition().set(0.003548, -0.000499, 0.192235);
      setCollisionGeometryObject(collisionBox);
   }
}
