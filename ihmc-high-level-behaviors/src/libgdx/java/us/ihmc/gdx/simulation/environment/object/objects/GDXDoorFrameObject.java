package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXDoorFrameObject extends GDXEnvironmentObject
{
   public GDXDoorFrameObject()
   {
      create(GDXModelLoader.loadG3DModel("DoorFrame.g3dj"));
   }

   @Override
   public GDXDoorFrameObject duplicate()
   {
      return new GDXDoorFrameObject();
   }
}
