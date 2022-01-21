package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXSmallCinderBlockRoughed extends GDXEnvironmentObject
{
   public static final String NAME = "Small Cinder Block Roughed";

   public GDXSmallCinderBlockRoughed()
   {
      super(NAME);
      create(GDXModelLoader.loadG3DModel("SmallCinderBlockRough.g3dj"));
   }

   @Override
   public GDXSmallCinderBlockRoughed duplicate()
   {
      return new GDXSmallCinderBlockRoughed();
   }
}
