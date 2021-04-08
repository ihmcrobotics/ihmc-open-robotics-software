package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXDoorFrameObject extends GDXEnvironmentObject
{
   @Override
   public Model getModel()
   {
      return GDXModelLoader.loadG3DModel("DoorFrame.g3dj");
   }

   @Override
   public GDXDoorFrameObject duplicate()
   {
      return new GDXDoorFrameObject();
   }
}
