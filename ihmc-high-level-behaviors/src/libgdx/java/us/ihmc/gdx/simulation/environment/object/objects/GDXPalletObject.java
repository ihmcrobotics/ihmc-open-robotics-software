package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXPalletObject extends GDXEnvironmentObject
{
   public static final String NAME = "Pallet";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXPalletObject.class);

   public GDXPalletObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.loadG3DModel("environmentObjects/pallet/Pallet.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 1.21;
      double sizeY = 1.013;
      double sizeZ = 0.155;
      setMass(30.0f);
      Sphere3D boundingSphere = new Sphere3D(0.7);
      getBoundingSphere().setRadius(0.7);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      setBtCollisionShape(new btBoxShape(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f)));
   }
}
