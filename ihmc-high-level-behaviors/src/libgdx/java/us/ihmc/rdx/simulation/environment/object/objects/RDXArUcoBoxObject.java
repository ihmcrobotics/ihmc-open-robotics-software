package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.behaviors.simulation.RigidBodySceneObjectDefinitions;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class RDXArUcoBoxObject extends RDXEnvironmentObject
{
   public static final String NAME = "Box";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXArUcoBoxObject.class);

   public RDXArUcoBoxObject()
   {
      super(NAME, FACTORY);
      loadRealisticModel(RigidBodySceneObjectDefinitions.BOX_VISUAL_MODEL_FILE_PATH);

      getBoundingSphere().setRadius(0.5);
      setMass(0.3f);
      Box3D collisionBox = new Box3D(RigidBodySceneObjectDefinitions.BOX_DEPTH,
                                     RigidBodySceneObjectDefinitions.BOX_WIDTH,
                                     RigidBodySceneObjectDefinitions.BOX_HEIGHT);
      setCollisionGeometryObject(collisionBox);
   }
}
