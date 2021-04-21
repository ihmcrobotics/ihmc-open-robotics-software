package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXMediumCinderBlockRoughed extends GDXEnvironmentObject
{
   public GDXMediumCinderBlockRoughed()
   {
      super(GDXModelLoader.loadG3DModel("MediumCinderBlockRough.g3dj"));
   }

   @Override
   public GDXMediumCinderBlockRoughed duplicate()
   {
      return new GDXMediumCinderBlockRoughed();
   }
}
