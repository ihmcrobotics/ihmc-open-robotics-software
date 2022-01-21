package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXLargeCinderBlockRoughed extends GDXEnvironmentObject
{
   public static final String NAME = "Large Cinder Block Roughed";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXLargeCinderBlockRoughed.class);

   public GDXLargeCinderBlockRoughed()
   {
      super(NAME, FACTORY);
      create(GDXModelLoader.loadG3DModel("LargeCinderBlockRough.g3dj"));
   }
}
