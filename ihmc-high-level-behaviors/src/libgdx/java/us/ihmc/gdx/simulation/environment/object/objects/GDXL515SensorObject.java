package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXL515SensorObject extends GDXEnvironmentObject
{
   public static final String NAME = "L515 Sensor";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXL515SensorObject.class);

   public GDXL515SensorObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.loadG3DModel("environmentObjects/l515Sensor/L515Sensor.g3dj");
      setRealisticModel(realisticModel);

      getBoundingSphere().setRadius(0.2);
      double sizeX = 0.1;
      double sizeY = 0.1;
      double sizeZ = 0.04;
      setMass(0.3f);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
   }
}
