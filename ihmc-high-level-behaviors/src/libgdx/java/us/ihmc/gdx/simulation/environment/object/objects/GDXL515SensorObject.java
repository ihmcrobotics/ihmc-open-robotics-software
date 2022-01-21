package us.ihmc.gdx.simulation.environment.object.objects;

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
      create(GDXModelLoader.loadG3DModel("sensor_l515.g3dj"));
   }
}
