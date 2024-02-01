package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXArUcoBoxObject extends RDXEnvironmentObject
{
   public static final String NAME = "Box";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXArUcoBoxObject.class);

   public RDXArUcoBoxObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load(RigidBodySceneObjectDefinitions.BOX_VISUAL_MODEL_FILE_PATH);
      setRealisticModel(realisticModel);

      getBoundingSphere().setRadius(0.5);
      setMass(0.3f);
      Box3D collisionBox = new Box3D(RigidBodySceneObjectDefinitions.BOX_DEPTH,
                                     RigidBodySceneObjectDefinitions.BOX_WIDTH,
                                     RigidBodySceneObjectDefinitions.BOX_HEIGHT);
      setCollisionGeometryObject(collisionBox);

      getRealisticModelOffset().getTranslation().add(0.0, 0.0, -collisionBox.getSizeZ() / 2.0);
   }
}
