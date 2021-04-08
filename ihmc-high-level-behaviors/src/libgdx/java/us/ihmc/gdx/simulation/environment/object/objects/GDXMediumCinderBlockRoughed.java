package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXMediumCinderBlockRoughed extends GDXEnvironmentObject
{
   @Override
   public Model getModel()
   {
      return GDXModelLoader.loadG3DModel("MediumCinderBlockRough.g3dj");
   }

   @Override
   public GDXMediumCinderBlockRoughed duplicate()
   {
      return new GDXMediumCinderBlockRoughed();
   }
}
