package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXLargeCinderBlockRoughed extends GDXEnvironmentObject
{
   public GDXLargeCinderBlockRoughed()
   {
      create(GDXModelLoader.loadG3DModel("LargeCinderBlockRough.g3dj"));
   }

   @Override
   public GDXLargeCinderBlockRoughed duplicate()
   {
      return new GDXLargeCinderBlockRoughed();
   }
}
