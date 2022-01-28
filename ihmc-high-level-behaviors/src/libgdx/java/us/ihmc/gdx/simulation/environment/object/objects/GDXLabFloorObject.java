package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXLabFloorObject extends GDXEnvironmentObject
{
   public static final String NAME = "Lab Floor";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXLabFloorObject.class);

   public GDXLabFloorObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.loadG3DModel("environmentObjects/labFloor/LabFloor.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 0.3;
      double sizeY = 0.3;
      double sizeZ = 0.01;
      setMass(0.0f);
      getBoundingSphere().setRadius(0.7);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      setBtCollisionShape(new btBoxShape(new Vector3(10.0f, 10.0f, 0.0001f)));
   }
}
