package us.ihmc.gdx.simulation.environment.object;

import us.ihmc.gdx.simulation.environment.object.objects.*;
import us.ihmc.gdx.simulation.environment.object.objects.door.*;

import java.util.ArrayList;

public class GDXEnvironmentObjectLibrary
{
   private static final ArrayList<GDXEnvironmentObjectFactory> objectFactories = new ArrayList<>();
   static
   {
      objectFactories.add(GDXSmallCinderBlockRoughed.FACTORY);
      objectFactories.add(GDXMediumCinderBlockRoughed.FACTORY);
      objectFactories.add(GDXLargeCinderBlockRoughed.FACTORY);
      objectFactories.add(GDXLabFloorObject.FACTORY);
      objectFactories.add(GDXPalletObject.FACTORY);
      objectFactories.add(GDXStairsObject.FACTORY);
      objectFactories.add(GDXDoorFrameObject.FACTORY);
      objectFactories.add(GDXDoorPanelObject.FACTORY);
      objectFactories.add(GDXDoorLeverHandleObject.FACTORY);
      objectFactories.add(GDXDoorObject.FACTORY);
      objectFactories.add(GDXPointLightObject.FACTORY);
      objectFactories.add(GDXDirectionalLightObject.FACTORY);
      objectFactories.add(GDXL515SensorObject.FACTORY);
      objectFactories.add(GDXMultiBodySnakeObject.FACTORY);
   }

   public static ArrayList<GDXEnvironmentObjectFactory> getObjectFactories()
   {
      return objectFactories;
   }

   public static GDXEnvironmentObject loadBySimpleClassName(String objectClassName)
   {
      for (GDXEnvironmentObjectFactory objectFactory : objectFactories)
      {
         if (objectFactory.getClazz().getSimpleName().equals(objectClassName))
         {
            return objectFactory.getSupplier().get();
         }
      }

      throw new RuntimeException("Library does not contain object of the name: " + objectClassName);
   }
}
