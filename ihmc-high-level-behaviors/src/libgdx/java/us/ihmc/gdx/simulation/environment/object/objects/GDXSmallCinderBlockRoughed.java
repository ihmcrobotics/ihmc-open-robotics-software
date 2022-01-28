package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXSmallCinderBlockRoughed extends GDXEnvironmentObject
{
   public static final String NAME = "Small Cinder Block Roughed";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXSmallCinderBlockRoughed.class);

   public GDXSmallCinderBlockRoughed()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.loadG3DModel("environmentObjects/smallCinderBlockRoughed/SmallCinderBlockRoughed.g3dj");
      setRealisticModel(realisticModel);

      getBoundingSphere().setRadius(0.7);
      double sizeX = 0.393;
      double sizeY = 0.192;
      double sizeZ = 0.0884;
      setMass(5.0f);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      setBtCollisionShape(new btBoxShape(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f)));
   }
}
