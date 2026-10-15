package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class RDXArUcoBoxObject extends RDXEnvironmentObject
{
   public static final String NAME = "Box";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXArUcoBoxObject.class);

   public RDXArUcoBoxObject()
   {
      super(NAME, FACTORY);
      loadRealisticModel("environmentObjects/box/emptyBox.g3dj");

      getBoundingSphere().setRadius(0.5);
      setMass(0.3f);
      Box3D collisionBox = new Box3D(0.31, 0.394, 0.265);
      setCollisionGeometryObject(collisionBox);
   }
}
