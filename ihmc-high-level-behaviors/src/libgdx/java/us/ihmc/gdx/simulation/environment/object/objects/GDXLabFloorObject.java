package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXLabFloorObject extends GDXEnvironmentObject
{
   public GDXLabFloorObject()
   {
      create(GDXModelLoader.loadG3DModel("labFloor.g3dj"));
   }

   @Override
   public GDXLabFloorObject duplicate()
   {
      return new GDXLabFloorObject();
   }
}
