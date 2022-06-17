package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.simulation.environment.object.GDXSCS2EnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXSCS2EnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXSCS2SmallCinderBlockRoughed extends GDXSCS2EnvironmentObject
{
   public static final String NAME = "Small Cinder Block Roughed";
   public static final GDXSCS2EnvironmentObjectFactory FACTORY = new GDXSCS2EnvironmentObjectFactory(NAME, GDXSCS2SmallCinderBlockRoughed.class);

   public GDXSCS2SmallCinderBlockRoughed()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.load("environmentObjects/smallCinderBlockRoughed/SmallCinderBlockRoughed.g3dj");
      setRealisticModel(realisticModel);

      getBoundingSphere().setRadius(0.7);
      double sizeX = 0.393;
      double sizeY = 0.192;
      double sizeZ = 0.0884;
      setMass(5.0f);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
//      setBtCollisionShape(new btBoxShape(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f)));
   }
}
