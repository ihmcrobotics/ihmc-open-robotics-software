package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXSmallCinderBlockRoughed extends GDXEnvironmentObject
{
   public static final String NAME = "Small Cinder Block Roughed";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXSmallCinderBlockRoughed.class);

   public GDXSmallCinderBlockRoughed()
   {
      super(NAME, FACTORY);
      create(GDXModelLoader.loadG3DModel("SmallCinderBlockRough.g3dj"));
   }
}
