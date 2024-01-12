package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXLargeCinderBlockRoughed extends RDXEnvironmentObject
{
   public static final String NAME = "Large Cinder Block Roughed";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXLargeCinderBlockRoughed.class);

   public RDXLargeCinderBlockRoughed()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/largeCinderBlockRoughed/LargeCinderBlockRoughed.g3dj");
      setRealisticModel(realisticModel);

      getBoundingSphere().setRadius(0.7);
      double sizeX = 0.393;
      double sizeY = 0.19;
      double sizeZ = 0.192;
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      setMass(9.0f);
   }
}
