package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXSmallCinderBlockRoughed extends RDXEnvironmentObject
{
   public static final String NAME = "Small Cinder Block Roughed";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXSmallCinderBlockRoughed.class);

   public RDXSmallCinderBlockRoughed()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/smallCinderBlockRoughed/SmallCinderBlockRoughed.g3dj");
      setRealisticModel(realisticModel);

      getBoundingSphere().setRadius(0.7);
      double sizeX = 0.393;
      double sizeY = 0.192;
      double sizeZ = 0.0884;
      setMass(5.0f);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
   }
}
