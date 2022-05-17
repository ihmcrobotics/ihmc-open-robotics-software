package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXLargeCinderBlockRoughed extends GDXEnvironmentObject
{
   public static final String NAME = "Large Cinder Block Roughed";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXLargeCinderBlockRoughed.class);

   public GDXLargeCinderBlockRoughed()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.load("environmentObjects/largeCinderBlockRoughed/LargeCinderBlockRoughed.g3dj");
      setRealisticModel(realisticModel);

      getBoundingSphere().setRadius(0.7);
      double sizeX = 0.393;
      double sizeY = 0.19;
      double sizeZ = 0.192;
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      setMass(9.0f);
      setBtCollisionShape(new btBoxShape(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f)));
   }
}
