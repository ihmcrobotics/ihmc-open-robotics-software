package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXDoorOnlyObject extends GDXEnvironmentObject
{
   public GDXDoorOnlyObject()
   {
      create(GDXModelLoader.loadG3DModel("DoorOnly.g3dj"));
   }

   @Override
   public GDXDoorOnlyObject duplicate()
   {
      return new GDXDoorOnlyObject();
   }
}
