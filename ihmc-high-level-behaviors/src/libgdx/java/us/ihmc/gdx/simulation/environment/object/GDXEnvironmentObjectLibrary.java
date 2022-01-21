package us.ihmc.gdx.simulation.environment.object;

import us.ihmc.gdx.simulation.environment.object.objects.*;

import java.util.ArrayList;

public class GDXEnvironmentObjectLibrary
{
   private static final ArrayList<GDXEnvironmentObjectFactory> objectFactories = new ArrayList<>();
   static
   {
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXSmallCinderBlockRoughed.NAME, GDXSmallCinderBlockRoughed.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXMediumCinderBlockRoughed.NAME, GDXMediumCinderBlockRoughed.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXLargeCinderBlockRoughed.NAME, GDXLargeCinderBlockRoughed.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXLabFloorObject.NAME, GDXLabFloorObject.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXPalletObject.NAME, GDXPalletObject.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXStairsObject.NAME, GDXStairsObject.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXPushHandleRightDoorObject.NAME, GDXPushHandleRightDoorObject.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXDoorFrameObject.NAME, GDXDoorFrameObject.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXPointLightObject.NAME, GDXPointLightObject.class));
      objectFactories.add(new GDXEnvironmentObjectFactory(GDXDirectionalLightObject.NAME, GDXDirectionalLightObject.class));
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
