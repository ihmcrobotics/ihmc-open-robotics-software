package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXSCS2EnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXSCS2EnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXSCS2SmallCinderBlockRoughed extends RDXSCS2EnvironmentObject
{
   public static final String NAME = "Small Cinder Block Roughed";
   public static final RDXSCS2EnvironmentObjectFactory FACTORY = new RDXSCS2EnvironmentObjectFactory(NAME, RDXSCS2SmallCinderBlockRoughed.class);

   public RDXSCS2SmallCinderBlockRoughed()
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
