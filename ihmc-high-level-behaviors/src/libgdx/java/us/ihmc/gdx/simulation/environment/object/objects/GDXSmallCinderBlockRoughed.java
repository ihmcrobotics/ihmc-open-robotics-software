package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXSmallCinderBlockRoughed extends GDXEnvironmentObject
{
   public GDXSmallCinderBlockRoughed()
   {
      super(GDXModelLoader.loadG3DModel("SmallCinderBlockRough.g3dj"));
   }

   @Override
   public GDXSmallCinderBlockRoughed duplicate()
   {
      return new GDXSmallCinderBlockRoughed();
   }
}
