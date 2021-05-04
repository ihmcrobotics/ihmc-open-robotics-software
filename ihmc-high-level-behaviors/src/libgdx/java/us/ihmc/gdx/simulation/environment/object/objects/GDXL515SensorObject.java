package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXL515SensorObject extends GDXEnvironmentObject
{
   public GDXL515SensorObject()
   {
      create(GDXModelLoader.loadG3DModel("sensor_l515.g3dj"));
   }

   @Override
   public GDXL515SensorObject duplicate()
   {
      return new GDXL515SensorObject();
   }
}
