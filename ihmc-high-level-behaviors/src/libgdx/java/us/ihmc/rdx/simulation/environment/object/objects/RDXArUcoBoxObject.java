package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.perception.sceneGraph.rigidBodies.RigidBodySceneObjectDefinitions;
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

      double size = 0.35;
      getBoundingSphere().setRadius(0.5);
      setMass(0.3f);
      Box3D collisionBox = new Box3D(size, size, size);
      setCollisionGeometryObject(collisionBox);

      getRealisticModelOffset().getTranslation().add(-size / 2.0, -size / 2.0, -size / 2.0);
   }
}
