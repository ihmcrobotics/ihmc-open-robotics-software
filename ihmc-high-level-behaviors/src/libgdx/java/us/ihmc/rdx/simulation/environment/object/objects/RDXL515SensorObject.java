package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXL515SensorObject extends RDXEnvironmentObject
{
   public static final String NAME = "L515 Sensor";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXL515SensorObject.class);

   public RDXL515SensorObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/l515Sensor/L515Sensor.g3dj");
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
